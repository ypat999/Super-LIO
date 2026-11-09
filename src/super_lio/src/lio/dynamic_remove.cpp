/**
 * @file dynamic_remove.cpp
 * @brief Dynamic point removal from point cloud frames
 * 
 * This module provides two methods for dynamic point removal:
 * 
 * Method 1 (TEMPORAL): Builds occupancy grids for each frame and filters out points
 * that are not occupied in both previous and next frames.
 * 
 * Method 2 (RAYCAST): Merges all frames first, then for each frame, performs raycast
 * from sensor position to each point. Marks voxels that are penetrated by rays and
 * removes those points from the merged cloud.
 */

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <fstream>
#include <sstream>
#include <mutex>
#include <memory>
#include <deque>
#include <limits>
#include <stdexcept>
#include <cstring>
#include <cctype>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>

namespace fs = std::filesystem;

namespace DynamicRemove {

enum class RemovalMethod {
    TEMPORAL = 0,
    RAYCAST = 1
};

struct Config {
    float grid_size = 0.2f;
    int min_neighbor_grids = 2;
    int frame_window = 1;
    std::string input_dir = "./PCD";
    std::string output_file = "./filtered_map.pcd";
    std::string output_dir = "";          // if set, save per-frame filtered PCDs here (prefix "filtered_")
    bool enable_isolated_removal = true;
    bool verbose = true;
    RemovalMethod method = RemovalMethod::TEMPORAL;
    int raycast_min_hits = 2;
    std::string scans_prefix = "";
    bool single_core = false;
};

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
    }
};

struct OdomData {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};

inline VoxelKey pointToVoxelKey(const PointType& point, float grid_size) {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.x / grid_size));
    key.y = static_cast<int>(std::floor(point.y / grid_size));
    key.z = static_cast<int>(std::floor(point.z / grid_size));
    return key;
}

inline VoxelKey positionToVoxelKey(const Eigen::Vector3f& pos, float grid_size) {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(pos.x() / grid_size));
    key.y = static_cast<int>(std::floor(pos.y() / grid_size));
    key.z = static_cast<int>(std::floor(pos.z() / grid_size));
    return key;
}

class OccupancyGrid {
public:
    explicit OccupancyGrid(float grid_size) : grid_size_(grid_size) {}

    void insertCloud(const CloudPtr& cloud) {
        for (const auto& point : cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            VoxelKey key = pointToVoxelKey(point, grid_size_);
            occupied_voxels_.insert(key);
        }
    }

    bool isOccupied(const VoxelKey& key) const {
        return occupied_voxels_.find(key) != occupied_voxels_.end();
    }

    const std::unordered_set<VoxelKey, VoxelKeyHash>& getOccupiedVoxels() const {
        return occupied_voxels_;
    }

    void clear() {
        occupied_voxels_.clear();
    }

    size_t size() const {
        return occupied_voxels_.size();
    }

private:
    float grid_size_;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
};

struct FrameData {
    CloudPtr cloud;
    OdomData odom;
    int index;
};

/// Prefix used for per-frame filtered outputs; must never be treated as input.
constexpr const char* kFilteredPrefix = "filtered_";

/// Reads the integer index out of a fragment name such as "scans_123.pcd".
/// Returns false when the file name carries no trailing number.
bool trailingIndex(const std::string& path, long long& index) {
    const std::string stem = fs::path(path).stem().string();
    size_t pos = stem.size();
    while (pos > 0 && std::isdigit(static_cast<unsigned char>(stem[pos - 1]))) {
        --pos;
    }
    if (pos == stem.size()) {
        return false;
    }
    index = std::stoll(stem.substr(pos));
    return true;
}

/// Orders fragments by their embedded index so that consecutive files are really
/// consecutive in time. Plain lexicographic order would place "scans_10" next to
/// "scans_100", which silently breaks the temporal window.
void sortFramesByIndex(std::vector<std::string>& paths) {
    std::sort(paths.begin(), paths.end(), [](const std::string& a, const std::string& b) {
        long long ia = 0, ib = 0;
        const bool oka = trailingIndex(a, ia);
        const bool okb = trailingIndex(b, ib);
        if (oka && okb && ia != ib) {
            return ia < ib;
        }
        return a < b;
    });
}

/// Lists input fragments in `input_dir`, honouring `scans_prefix` and always
/// skipping previously generated "filtered_*" outputs (input_dir may equal output_dir).
std::vector<std::string> collectFrameFiles(const std::string& input_dir,
                                           const std::string& scans_prefix,
                                           bool verbose) {
    std::vector<std::string> pcd_files;
    if (!fs::exists(input_dir)) {
        LOG(ERROR) << "Input directory does not exist: " << input_dir;
        return pcd_files;
    }

    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.path().extension() != ".pcd") {
            continue;
        }
        std::string filename = entry.path().filename().string();
        if (filename.compare(0, std::strlen(kFilteredPrefix), kFilteredPrefix) == 0) {
            continue;
        }
        if (scans_prefix.empty() || filename.find(scans_prefix) == 0) {
            pcd_files.push_back(entry.path().string());
        }
    }

    sortFramesByIndex(pcd_files);

    if (verbose) {
        LOG(INFO) << "Found " << pcd_files.size() << " PCD files in " << input_dir
                  << (scans_prefix.empty() ? "" : " with prefix '" + scans_prefix + "'");
    }

    return pcd_files;
}

/// Loads a single fragment, dropping non-finite points. Returns false on read error.
bool loadSingleFrame(const std::string& path, CloudPtr& cloud, bool verbose) {
    cloud.reset(new PointCloudType());
    PointCloudType raw;
    if (pcl::io::loadPCDFile<PointType>(path, raw) != 0) {
        LOG(WARNING) << "Failed to load: " << path;
        return false;
    }

    CloudPtr valid_cloud(new PointCloudType());
    valid_cloud->reserve(raw.size());
    for (const auto& pt : raw.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            valid_cloud->points.push_back(pt);
        }
    }
    valid_cloud->width = valid_cloud->points.size();
    valid_cloud->height = 1;
    valid_cloud->is_dense = true;
    cloud = valid_cloud;

    if (verbose) {
        LOG(INFO) << "Loaded: " << path << " (" << cloud->size() << " points)";
    }
    return true;
}

/// Saves one fragment; PCL throws on empty clouds, so those are skipped with a warning.
/// Returns true when a file was written.
bool saveFrameCloud(const std::string& path, const CloudPtr& cloud, bool verbose) {
    if (cloud->empty()) {
        LOG(WARNING) << "Skipped (no points left after filtering): " << path;
        return false;
    }
    try {
        pcl::io::savePCDFileBinary(path, *cloud);
    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to save " << path << ": " << e.what();
        return false;
    }
    if (verbose) {
        LOG(INFO) << "Saved filtered frame: " << path << " (" << cloud->size() << " points)";
    }
    return true;
}

std::vector<FrameData> loadPointCloudFramesWithOdom(const std::string& input_dir, bool verbose, std::vector<std::string>* out_filenames = nullptr) {
    std::vector<FrameData> frames;
    std::vector<std::pair<std::string, std::string>> pcd_txt_files;

    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.path().extension() == ".pcd") {
            std::string filename = entry.path().filename().string();
            if (filename.compare(0, std::strlen(kFilteredPrefix), kFilteredPrefix) == 0) {
                continue;
            }
            std::string pcd_file = entry.path().string();
            std::string txt_file = pcd_file.substr(0, pcd_file.size() - 4) + ".txt";
            
            if (fs::exists(txt_file)) {
                pcd_txt_files.push_back({pcd_file, txt_file});
            } else {
                if (verbose) {
                    LOG(WARNING) << "No odom file for: " << pcd_file;
                }
            }
        }
    }

    std::sort(pcd_txt_files.begin(), pcd_txt_files.end(), 
        [](const auto& a, const auto& b) {
            long long ia = 0, ib = 0;
            const bool oka = trailingIndex(a.first, ia);
            const bool okb = trailingIndex(b.first, ib);
            if (oka && okb && ia != ib) {
                return ia < ib;
            }
            return a.first < b.first;
        });

    if (verbose) {
        LOG(INFO) << "Found " << pcd_txt_files.size() << " PCD files with odom in " << input_dir;
    }

    int index = 0;
    for (const auto& [pcd_file, txt_file] : pcd_txt_files) {
        CloudPtr cloud(new PointCloudType());
        if (pcl::io::loadPCDFile<PointType>(pcd_file, *cloud) == 0) {
            CloudPtr valid_cloud(new PointCloudType());
            for (const auto& pt : cloud->points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    valid_cloud->points.push_back(pt);
                }
            }
            valid_cloud->width = valid_cloud->points.size();
            valid_cloud->height = 1;
            valid_cloud->is_dense = true;

            OdomData odom;
            std::ifstream odom_stream(txt_file);
            if (odom_stream.is_open()) {
                float qx, qy, qz, qw;
                odom_stream >> odom.timestamp 
                           >> odom.position.x() >> odom.position.y() >> odom.position.z()
                           >> qx >> qy >> qz >> qw;
                odom.orientation = Eigen::Quaternionf(qw, qx, qy, qz);
                odom.orientation.normalize();
                odom_stream.close();
            } else {
                LOG(WARNING) << "Failed to read odom: " << txt_file;
                continue;
            }

            FrameData frame;
            frame.cloud = valid_cloud;
            frame.odom = odom;
            frame.index = index++;
            frames.push_back(frame);

            if (out_filenames) {
                out_filenames->push_back(pcd_file);
            }

            if (verbose) {
                LOG(INFO) << "Loaded: " << pcd_file << " (" << valid_cloud->size() 
                          << " points, odom: " << odom.position.transpose() << ")";
            }
        } else {
            LOG(WARNING) << "Failed to load: " << pcd_file;
        }
    }

    return frames;
}

/// Loads every fragment at once. Only used by the legacy single-file output mode,
/// which has to keep all kept points in RAM for the global isolated-point pass.
std::vector<CloudPtr> loadPointCloudFrames(const std::string& input_dir, bool verbose, const std::string& scans_prefix = "", std::vector<std::string>* out_filenames = nullptr) {
    std::vector<CloudPtr> frames;
    const std::vector<std::string> pcd_files = collectFrameFiles(input_dir, scans_prefix, verbose);

    for (const auto& file : pcd_files) {
        CloudPtr cloud;
        if (!loadSingleFrame(file, cloud, verbose)) {
            continue;
        }
        frames.push_back(cloud);
        if (out_filenames) {
            out_filenames->push_back(file);
        }
    }

    return frames;
}

/// Legacy single-file path: keeps every frame plus every frame's occupancy grid
/// alive at the same time, so its peak memory grows with the number of fragments.
/// Prefer --output_dir (see runTemporalStreaming) for large maps.
CloudPtr filterDynamicPointsTemporal(
    const std::vector<CloudPtr>& frames,
    const Config& config)
{
    if (frames.empty()) {
        LOG(WARNING) << "No frames to process";
        return CloudPtr(new PointCloudType());
    }

    size_t n_frames = frames.size();
    std::vector<OccupancyGrid> grids(n_frames, OccupancyGrid(config.grid_size));

    for (size_t i = 0; i < n_frames; ++i) {
        grids[i].insertCloud(frames[i]);
        if (config.verbose && i % 10 == 0) {
            LOG(INFO) << "Frame " << i << ": " << grids[i].size() << " occupied voxels";
        }
    }

    std::vector<PointType, Eigen::aligned_allocator<PointType>> filtered_points;
    std::vector<int> frame_removed_counts(n_frames, 0);
    size_t total_points = 0;
    size_t removed_points = 0;

    for (size_t frame_idx = 0; frame_idx < n_frames; ++frame_idx) {
        const CloudPtr& frame = frames[frame_idx];
        int frame_removed = 0;

        for (const auto& point : frame->points) {
            total_points++;
            VoxelKey key = pointToVoxelKey(point, config.grid_size);

            bool is_dynamic = true;
            int window = config.frame_window;

            for (int offset = -window; offset <= window; ++offset) {
                if (offset == 0) continue;
                
                int neighbor_idx = static_cast<int>(frame_idx) + offset;
                if (neighbor_idx < 0 || neighbor_idx >= static_cast<int>(n_frames)) {
                    continue;
                }

                if (grids[neighbor_idx].isOccupied(key)) {
                    is_dynamic = false;
                    break;
                }
            }

            if (is_dynamic) {
                removed_points++;
                frame_removed++;
            } else {
                filtered_points.push_back(point);
            }
        }

        frame_removed_counts[frame_idx] = frame_removed;
    }

    for (size_t frame_idx = 0; frame_idx < n_frames; ++frame_idx) {
        if (config.verbose && frame_removed_counts[frame_idx] > 0) {
            LOG(INFO) << "Frame " << frame_idx << ": removed " << frame_removed_counts[frame_idx] << " dynamic points";
        }
    }

    if (config.verbose) {
        LOG(INFO) << "Temporal dynamic removal: " << removed_points << " / " << total_points 
                  << " points removed (" << (100.0 * removed_points / total_points) << "%)";
    }

    CloudPtr filtered_cloud(new PointCloudType());
    filtered_cloud->points = std::move(filtered_points);
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

std::vector<VoxelKey> raycastVoxels(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    float grid_size)
{
    std::vector<VoxelKey> voxels;
    
    VoxelKey start_key = positionToVoxelKey(start, grid_size);
    VoxelKey end_key = positionToVoxelKey(end, grid_size);
    
    int dx = end_key.x - start_key.x;
    int dy = end_key.y - start_key.y;
    int dz = end_key.z - start_key.z;
    
    int step_x = (dx >= 0) ? 1 : -1;
    int step_y = (dy >= 0) ? 1 : -1;
    int step_z = (dz >= 0) ? 1 : -1;
    
    int abs_dx = std::abs(dx);
    int abs_dy = std::abs(dy);
    int abs_dz = std::abs(dz);
    
    int max_dist = abs_dx + abs_dy + abs_dz;
    if (max_dist == 0) return voxels;
    
    double t_delta_x = (dx != 0) ? (double)grid_size / std::abs(end.x() - start.x()) : 1e10;
    double t_delta_y = (dy != 0) ? (double)grid_size / std::abs(end.y() - start.y()) : 1e10;
    double t_delta_z = (dz != 0) ? (double)grid_size / std::abs(end.z() - start.z()) : 1e10;
    
    double t_max_x = (dx != 0) ? 
        ((start_key.x + step_x) * grid_size - start.x()) / (end.x() - start.x()) : 1e10;
    double t_max_y = (dy != 0) ? 
        ((start_key.y + step_y) * grid_size - start.y()) / (end.y() - start.y()) : 1e10;
    double t_max_z = (dz != 0) ? 
        ((start_key.z + step_z) * grid_size - start.z()) / (end.z() - start.z()) : 1e10;
    
    int x = start_key.x;
    int y = start_key.y;
    int z = start_key.z;
    
    for (int i = 0; i < max_dist * 2 + 1; ++i) {
        if (x != end_key.x || y != end_key.y || z != end_key.z) {
            voxels.push_back({x, y, z});
        }
        
        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                x += step_x;
                t_max_x += t_delta_x;
            } else {
                z += step_z;
                t_max_z += t_delta_z;
            }
        } else {
            if (t_max_y < t_max_z) {
                y += step_y;
                t_max_y += t_delta_y;
            } else {
                z += step_z;
                t_max_z += t_delta_z;
            }
        }
        
        if (t_max_x > 1.0 && t_max_y > 1.0 && t_max_z > 1.0) break;
    }
    
    return voxels;
}

CloudPtr filterDynamicPointsRaycast(
    const std::vector<FrameData>& frames,
    const Config& config)
{
    if (frames.empty()) {
        LOG(WARNING) << "No frames to process";
        return CloudPtr(new PointCloudType());
    }

    LOG(INFO) << "=== Raycast Method: Building global occupancy grid ===";
    
    CloudPtr merged_cloud(new PointCloudType());
    for (const auto& frame : frames) {
        *merged_cloud += *frame.cloud;
    }
    
    OccupancyGrid global_grid(config.grid_size);
    global_grid.insertCloud(merged_cloud);
    
    if (config.verbose) {
        LOG(INFO) << "Merged cloud: " << merged_cloud->size() << " points";
        LOG(INFO) << "Global grid: " << global_grid.size() << " occupied voxels";
    }

    LOG(INFO) << "=== Raycast Method: Counting observations and penetrations ===";
    
    std::unordered_map<VoxelKey, int, VoxelKeyHash> observation_count;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> penetration_count;
    std::mutex obs_mutex, pen_mutex;
    
    const auto& occupied_voxels = global_grid.getOccupiedVoxels();
    
    for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
        const auto& frame = frames[frame_idx];
        const Eigen::Vector3f& sensor_pos = frame.odom.position;
        VoxelKey sensor_key = positionToVoxelKey(sensor_pos, config.grid_size);
        
        int frame_penetrations = 0;
        
        std::unordered_map<VoxelKey, int, VoxelKeyHash> local_obs;
        std::unordered_map<VoxelKey, int, VoxelKeyHash> local_pen;
        
        for (const auto& point : frame.cloud->points) {
            Eigen::Vector3f point_pos(point.x, point.y, point.z);
            VoxelKey point_key = pointToVoxelKey(point, config.grid_size);
            
            local_obs[point_key]++;
            
            if (point_key.x == sensor_key.x && point_key.y == sensor_key.y && point_key.z == sensor_key.z) {
                continue;
            }
            
            std::vector<VoxelKey> ray_voxels = raycastVoxels(sensor_pos, point_pos, config.grid_size);
            
            for (const auto& voxel_key : ray_voxels) {
                if (occupied_voxels.find(voxel_key) != occupied_voxels.end()) {
                    local_pen[voxel_key]++;
                    frame_penetrations++;
                }
            }
        }
        
        for (const auto& [key, count] : local_obs) {
            observation_count[key] += count;
        }
        for (const auto& [key, count] : local_pen) {
            penetration_count[key] += count;
        }
        
        if (config.verbose) {
            LOG(INFO) << "Frame " << frame_idx << ": " << local_obs.size() 
                    << " observed, " << frame_penetrations << " penetrations";
        }
    }

    LOG(INFO) << "=== Raycast Method: Filtering dynamic voxels ===";
    
    std::unordered_set<VoxelKey, VoxelKeyHash> dynamic_voxels;
    int total_checked = 0;
    
    for (const auto& voxel_key : occupied_voxels) {
        int obs = observation_count[voxel_key];
        int pen = penetration_count[voxel_key];
        
        if (obs > 0) {
            total_checked++;
            float ratio = static_cast<float>(pen) / static_cast<float>(obs);
            
            if (pen >= config.raycast_min_hits && ratio > 0.5f) {
                dynamic_voxels.insert(voxel_key);
            }
        }
    }
    
    if (config.verbose) {
        LOG(INFO) << "Dynamic voxels (pen>=" << config.raycast_min_hits 
                  << ", ratio>0.5): " << dynamic_voxels.size();
    }

    CloudPtr filtered_cloud(new PointCloudType());
    int total_points = merged_cloud->size();
    int removed_points = 0;
    
    for (const auto& point : merged_cloud->points) {
        VoxelKey key = pointToVoxelKey(point, config.grid_size);
        
        if (dynamic_voxels.find(key) != dynamic_voxels.end()) {
            removed_points++;
        } else {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    if (config.verbose) {
        LOG(INFO) << "Raycast dynamic removal: " << removed_points << " / " << total_points 
                  << " points removed (" << (100.0 * removed_points / total_points) << "%)";
    }
    
    return filtered_cloud;
}

CloudPtr removeIsolatedPoints(
    const CloudPtr& cloud,
    const Config& config)
{
    if (!config.enable_isolated_removal || cloud->empty()) {
        return cloud;
    }

    OccupancyGrid grid(config.grid_size);
    grid.insertCloud(cloud);

    CloudPtr filtered_cloud(new PointCloudType());
    int total_points = cloud->size();
    int isolated_points = 0;

    const std::vector<std::tuple<int, int, int>> neighbor_offsets = {
        {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1},
        {-1, 0, -1},  {-1, 0, 0},  {-1, 0, 1},
        {-1, 1, -1},  {-1, 1, 0},  {-1, 1, 1},
        {0, -1, -1},  {0, -1, 0},  {0, -1, 1},
        {0, 0, -1},   {0, 0, 1},
        {0, 1, -1},   {0, 1, 0},   {0, 1, 1},
        {1, -1, -1},  {1, -1, 0},  {1, -1, 1},
        {1, 0, -1},   {1, 0, 0},   {1, 0, 1},
        {1, 1, -1},   {1, 1, 0},   {1, 1, 1}
    };

    for (const auto& point : cloud->points) {
        VoxelKey key = pointToVoxelKey(point, config.grid_size);
        
        int neighbor_count = 0;
        for (const auto& [dx, dy, dz] : neighbor_offsets) {
            VoxelKey neighbor_key{key.x + dx, key.y + dy, key.z + dz};
            if (grid.isOccupied(neighbor_key)) {
                neighbor_count++;
            }
        }

        if (neighbor_count < config.min_neighbor_grids) {
            isolated_points++;
        } else {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    if (config.verbose) {
        LOG(INFO) << "Isolated removal: " << isolated_points << " / " << total_points 
                  << " points removed (" << (100.0 * isolated_points / total_points) << "%)";
    }

    return filtered_cloud;
}

/// Streaming temporal filter for the per-frame output mode.
///
/// A point in frame i survives when its voxel is also occupied by any frame inside
/// [i - window, i + window] except i itself, so only 2*window + 1 grids are ever
/// needed. Grids are built on a sliding ring and frames are written out one at a
/// time, which keeps peak memory independent of the number of fragments.
/// Returns false when the run could not be started.
bool runTemporalStreaming(const Config& config, const std::vector<std::string>& files) {
    const size_t n_frames = files.size();
    if (n_frames == 0) {
        LOG(ERROR) << "No valid point cloud frames found. Exiting.";
        return false;
    }

    const size_t window = static_cast<size_t>(std::max(0, config.frame_window));

    if (!fs::exists(config.output_dir)) {
        fs::create_directories(config.output_dir);
    }

    // ring[k] is the occupancy grid of frame (ring_front + k).
    std::deque<std::unique_ptr<OccupancyGrid>> ring;
    size_t ring_front = 0;

    // An unreadable frame contributes an empty grid, so the ring stays index-aligned.
    auto append_grid = [&](size_t frame_idx) {
        CloudPtr cloud;
        if (!loadSingleFrame(files[frame_idx], cloud, false)) {
            LOG(WARNING) << "Unreadable frame, treated as empty: " << files[frame_idx];
        }
        ring.push_back(std::unique_ptr<OccupancyGrid>(new OccupancyGrid(config.grid_size)));
        ring.back()->insertCloud(cloud);
    };

    size_t total_points = 0;
    size_t removed_points = 0;
    int saved_count = 0;

    for (size_t i = 0; i < n_frames; ++i) {
        // Read ahead until the grid covering the last frame of the window exists.
        const size_t want = std::min(n_frames - 1, i + window);
        while (ring_front + ring.size() <= want) {
            append_grid(ring_front + ring.size());
        }

        // Drop grids that fell out of the left edge of the window.
        while (!ring.empty() && i > window && ring_front < i - window) {
            ring.pop_front();
            ++ring_front;
        }

        CloudPtr frame;
        if (!loadSingleFrame(files[i], frame, config.verbose)) {
            continue;
        }

        CloudPtr kept(new PointCloudType());
        kept->reserve(frame->size());
        size_t frame_removed = 0;

        for (const auto& point : frame->points) {
            ++total_points;
            const VoxelKey key = pointToVoxelKey(point, config.grid_size);

            bool is_dynamic = true;
            if (ring.empty()) {
                // No neighbouring frame available at all: nothing to confirm, keep the point.
                is_dynamic = false;
            } else {
                const size_t lo = (i > window) ? (i - window) : 0;
                const size_t hi = std::min(n_frames - 1, i + window);
                for (size_t j = lo; j <= hi; ++j) {
                    if (j == i) continue;
                    if (j < ring_front || j >= ring_front + ring.size()) continue;
                    if (ring[j - ring_front]->isOccupied(key)) {
                        is_dynamic = false;
                        break;
                    }
                }
            }

            if (is_dynamic) {
                ++removed_points;
                ++frame_removed;
            } else {
                kept->points.push_back(point);
            }
        }

        kept->width = kept->points.size();
        kept->height = 1;
        kept->is_dense = true;

        if (config.enable_isolated_removal) {
            kept = removeIsolatedPoints(kept, config);
        }

        if (config.verbose && frame_removed > 0) {
            LOG(INFO) << "Frame " << i << ": removed " << frame_removed << " dynamic points";
        }

        const std::string filtered_name = config.output_dir + "/" + kFilteredPrefix
                                        + fs::path(files[i]).filename().string();
        if (saveFrameCloud(filtered_name, kept, config.verbose)) {
            ++saved_count;
        }
    }

    if (total_points > 0) {
        LOG(INFO) << "Temporal dynamic removal: " << removed_points << " / " << total_points
                  << " points removed (" << (100.0 * removed_points / total_points) << "%)";
    }
    LOG(INFO) << "Per-frame output: " << saved_count << " filtered PCDs saved to " << config.output_dir;
    return true;
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "Options:\n"
              << "  --input_dir <path>       Input directory containing PCD files (default: ./PCD)\n"
              << "  --output_file <path>     Output PCD file path (default: ./filtered_map.pcd)\n"
              << "  --output_dir <path>      Output per-frame filtered PCDs to directory (prefix 'filtered_')\n"
              << "                           Memory-bounded streaming mode; recommended for large maps.\n"
              << "  --grid_size <float>      Voxel grid size in meters (default: 0.2)\n"
              << "  --min_neighbors <int>    Minimum neighbor grids to keep a point (default: 2)\n"
              << "  --frame_window <int>     Frame window size for temporal method (default: 1)\n"
              << "  --method <0|1>           Removal method: 0=Temporal, 1=Raycast (default: 0)\n"
              << "  --raycast_min_hits <int> Min hits for raycast method (default: 2)\n"
              << "  --scans_prefix <str>     Only process PCD files with this prefix (default: empty)\n"
              << "  --disable_isolated       Disable isolated point removal\n"
              << "  --quiet                  Reduce output verbosity\n"
              << "  --single_core            Single-core mode (disable parallelism)\n"
              << "  --help                   Show this help message\n";
}

Config parseArgs(int argc, char** argv) {
    Config config;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            printUsage(argv[0]);
            exit(0);
        } else if (arg == "--input_dir" && i + 1 < argc) {
            config.input_dir = argv[++i];
        } else if (arg == "--output_file" && i + 1 < argc) {
            config.output_file = argv[++i];
        } else if (arg == "--output_dir" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if (arg == "--grid_size" && i + 1 < argc) {
            config.grid_size = std::stof(argv[++i]);
        } else if (arg == "--min_neighbors" && i + 1 < argc) {
            config.min_neighbor_grids = std::stoi(argv[++i]);
        } else if (arg == "--frame_window" && i + 1 < argc) {
            config.frame_window = std::stoi(argv[++i]);
        } else if (arg == "--method" && i + 1 < argc) {
            int method = std::stoi(argv[++i]);
            config.method = (method == 1) ? RemovalMethod::RAYCAST : RemovalMethod::TEMPORAL;
        } else if (arg == "--raycast_min_hits" && i + 1 < argc) {
            config.raycast_min_hits = std::stoi(argv[++i]);
        } else if (arg == "--scans_prefix" && i + 1 < argc) {
            config.scans_prefix = argv[++i];
        } else if (arg == "--disable_isolated") {
            config.enable_isolated_removal = false;
        } else if (arg == "--quiet") {
            config.verbose = false;
        } else if (arg == "--single_core") {
            config.single_core = true;
        } else {
            LOG(WARNING) << "Unknown argument: " << arg;
        }
    }
    
    return config;
}

void runDynamicRemoval(const Config& config) {
    LOG(INFO) << "=== Dynamic Point Removal Configuration ===";
    LOG(INFO) << "Input directory: " << config.input_dir;
    if (!config.output_dir.empty()) {
        LOG(INFO) << "Output dir (per-frame): " << config.output_dir;
    } else {
        LOG(INFO) << "Output file: " << config.output_file;
    }
    LOG(INFO) << "Grid size: " << config.grid_size << " m";
    LOG(INFO) << "Min neighbor grids: " << config.min_neighbor_grids;
    LOG(INFO) << "Method: " << (config.method == RemovalMethod::TEMPORAL ? "Temporal" : "Raycast");
    LOG(INFO) << "Single-core mode: " << (config.single_core ? "true" : "false");
    
    if (config.method == RemovalMethod::TEMPORAL) {
        LOG(INFO) << "Frame window: " << config.frame_window;
    } else {
        LOG(INFO) << "Raycast min hits: " << config.raycast_min_hits;
    }
    LOG(INFO) << "Isolated removal: " << (config.enable_isolated_removal ? "enabled" : "disabled");

    bool per_frame_mode = !config.output_dir.empty();

    if (config.method == RemovalMethod::RAYCAST) {
        LOG(INFO) << "\n=== Loading Point Cloud Frames with Odom ===";
        std::vector<std::string> filenames;
        std::vector<FrameData> frames = loadPointCloudFramesWithOdom(config.input_dir, config.verbose,
                                                                      per_frame_mode ? &filenames : nullptr);
        
        if (frames.empty()) {
            LOG(ERROR) << "No valid point cloud frames with odom loaded. Exiting.";
            return;
        }

        LOG(INFO) << "\n=== Filtering Dynamic Points (Raycast Method) ===";
        CloudPtr filtered_cloud = filterDynamicPointsRaycast(frames, config);

        if (per_frame_mode) {
            // Per-frame mode for raycast: use the merged filtered cloud to identify which voxels were kept,
            // then re-scan the original frames and save per-frame filtered PCDs.
            // Build occupancy grid of kept voxels from the filtered cloud.
            LOG(INFO) << "\n=== Per-frame output (Raycast): splitting filtered cloud back to per-frame ===";
            OccupancyGrid kept_grid(config.grid_size);
            kept_grid.insertCloud(filtered_cloud);

            if (!fs::exists(config.output_dir)) {
                fs::create_directories(config.output_dir);
            }
            int saved_count = 0;
            for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
                CloudPtr pf_cloud(new PointCloudType());
                for (const auto& point : frames[frame_idx].cloud->points) {
                    VoxelKey key = pointToVoxelKey(point, config.grid_size);
                    if (kept_grid.isOccupied(key)) {
                        pf_cloud->points.push_back(point);
                    }
                }
                pf_cloud->width = pf_cloud->points.size();
                pf_cloud->height = 1;
                pf_cloud->is_dense = true;

                // Apply isolated removal per-frame if enabled
                if (config.enable_isolated_removal) {
                    pf_cloud = removeIsolatedPoints(pf_cloud, config);
                }

                std::string basename;
                if (frame_idx < filenames.size()) {
                  basename = fs::path(filenames[frame_idx]).filename().string();
                } else {
                  basename = "scans_" + std::to_string(frame_idx) + ".pcd";
                }
                std::string filtered_name = std::string(config.output_dir + "/") + kFilteredPrefix + basename;
                if (saveFrameCloud(filtered_name, pf_cloud, false)) {
                    saved_count++;
                }
            }
            LOG(INFO) << "Per-frame output: " << saved_count << " filtered PCDs saved to " << config.output_dir;
            return;
        }

        // Original single-file output
        LOG(INFO) << "\n=== Removing Isolated Points ===";
        CloudPtr final_cloud = removeIsolatedPoints(filtered_cloud, config);
        LOG(INFO) << "\n=== Saving Result ===";
        if (final_cloud->empty()) {
            LOG(WARNING) << "No points remaining after filtering. Output file not created.";
            return;
        }
        if (pcl::io::savePCDFileBinary(config.output_file, *final_cloud) == 0) {
            LOG(INFO) << "Successfully saved filtered point cloud to: " << config.output_file;
            LOG(INFO) << "Final point count: " << final_cloud->size();
        } else {
            LOG(ERROR) << "Failed to save point cloud to: " << config.output_file;
        }
    } else {
        // TEMPORAL method
        if (per_frame_mode) {
            LOG(INFO) << "\n=== Filtering Dynamic Points (Temporal Method, streaming) ===";
            const std::vector<std::string> files =
                collectFrameFiles(config.input_dir, config.scans_prefix, config.verbose);
            runTemporalStreaming(config, files);
            return;
        }

        LOG(INFO) << "\n=== Loading Point Cloud Frames ===";
        LOG(WARNING) << "Single-file output keeps every frame and every frame grid in RAM; "
                     << "use --output_dir for a memory-bounded run.";
        std::vector<CloudPtr> frames = loadPointCloudFrames(config.input_dir, config.verbose,
                                                            config.scans_prefix);

        if (frames.empty()) {
            LOG(ERROR) << "No valid point cloud frames loaded. Exiting.";
            return;
        }

        LOG(INFO) << "\n=== Filtering Dynamic Points (Temporal Method) ===";
        CloudPtr filtered_cloud = filterDynamicPointsTemporal(frames, config);

        // Original single-file output
        LOG(INFO) << "\n=== Removing Isolated Points ===";
        CloudPtr final_cloud = removeIsolatedPoints(filtered_cloud, config);

        LOG(INFO) << "\n=== Saving Result ===";
        if (final_cloud->empty()) {
            LOG(WARNING) << "No points remaining after filtering. Output file not created.";
            return;
        }
        if (pcl::io::savePCDFileBinary(config.output_file, *final_cloud) == 0) {
            LOG(INFO) << "Successfully saved filtered point cloud to: " << config.output_file;
            LOG(INFO) << "Final point count: " << final_cloud->size();
        } else {
            LOG(ERROR) << "Failed to save point cloud to: " << config.output_file;
        }
    }
}

}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    DynamicRemove::Config config = DynamicRemove::parseArgs(argc, argv);
    DynamicRemove::runDynamicRemoval(config);

    google::ShutdownGoogleLogging();
    return 0;
}
