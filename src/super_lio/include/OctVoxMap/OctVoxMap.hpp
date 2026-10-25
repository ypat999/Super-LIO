
#ifndef OctVoxMap_HPP_
#define OctVoxMap_HPP_

#include <set>
#include <list>
#include <queue>
#include <vector>
#include <memory>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <execution>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>

#include "tsl/robin_map.h"
#include "HKNN_list60_gem.h"
#include "lio/params.h"


namespace LI2Sup{

template<int K, typename Point>
class KNNHeap {
public:
  KNNHeap() : count(0), worst_(0), max_dist2_(0.0f) {
    memset(dist2_, 0, sizeof(dist2_));
  }

  void reset() {
    count = 0;
    worst_ = 0;
    max_dist2_ = 0.0f;
    memset(dist2_, 0, sizeof(dist2_));
  }

  uint8_t count;
  uint8_t worst_;
  float max_dist2_;
  float dist2_[K];
  std::array<Point, K> points_;

  inline void try_insert(float dist2, const Point& pt) {
    const bool not_full = (count < K);
    const bool should_insert = not_full || (dist2 < max_dist2_);
    
    if (should_insert) {
      const uint8_t insert_idx = not_full ? count : worst_;
      
      dist2_[insert_idx] = dist2;
      points_[insert_idx] = pt;
      
      if (not_full) {
        count++;
        if (dist2 > max_dist2_) {
          max_dist2_ = dist2;
          worst_ = insert_idx;
        }
      } else {
        update_worst_unrolled();
      }
    }
  }

private:
  inline void update_worst_unrolled() {
    float d0 = dist2_[0], d1 = dist2_[1], d2 = dist2_[2], d3 = dist2_[3], d4 = dist2_[4];
    
    uint8_t idx01 = d0 > d1 ? 0 : 1;
    float max01 = d0 > d1 ? d0 : d1;
    
    uint8_t idx23 = d2 > d3 ? 2 : 3;
    float max23 = d2 > d3 ? d2 : d3;
    
    uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
    float max0123 = max01 > max23 ? max01 : max23;
    
    worst_ = max0123 > d4 ? idx0123 : 4;
    max_dist2_ = max0123 > d4 ? max0123 : d4;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
};


template<typename Point>
class OctVox{
public:
  OctVox(const Point& pt, uint8_t local_idx, const Point& origin = Point(0, 0, 0),
         float sub_res = 0.0f)
      : origin_(origin), sub_res_(sub_res)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  /// 子体素几何中心（由粗体素原点 + local_idx 推算）
  Point SubCenter(uint8_t local_idx) const {
    const float half = 0.5f * sub_res_;
    return Point(origin_.x() + ((local_idx & 1) ? 3.0f : 1.0f) * half,
                 origin_.y() + ((local_idx & 2) ? 3.0f : 1.0f) * half,
                 origin_.z() + ((local_idx & 4) ? 3.0f : 1.0f) * half);
  }

  void AddPoint(const Point& pt, uint8_t local_idx) {
    uint8_t& count = counts_[local_idx];
    Point& stored_point = points_[local_idx];
    if(count == UNINIT_MASK) {
      stored_point = pt;
      count = 1;
      return;
    }

    if(count >= g_map_max_points) return;
    const float merge_dist2 = static_cast<float>(g_map_merge_dist * g_map_merge_dist);
    if ((pt - stored_point).squaredNorm() > merge_dist2) return;

    if (g_map_rep_mode == 1) {
      // 代表点模式：只存真实扫描点，保留离子体素中心最近的那个
      const Point center = SubCenter(local_idx);
      if ((pt - center).squaredNorm() < (stored_point - center).squaredNorm()) {
        stored_point = pt;
      }
      ++count;
      return;
    }

    // 论文行为：增量均值
    stored_point = (stored_point * count + pt) / (count + 1);
    ++count;

    if (g_map_clamp_mu && sub_res_ > 0.0f) {
      // 均值可能被拉出子体素，按需夹回子体素包围盒
      const Point center = SubCenter(local_idx);
      const float half = 0.5f * sub_res_;
      for (int k = 0; k < 3; ++k) {
        stored_point[k] = std::min(std::max(stored_point[k], center[k] - half),
                                   center[k] + half);
      }
    }
  }

  bool getPoint(const uint8_t local_idx, Point& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  static constexpr uint8_t UNINIT_MASK = 0x00;

  std::array<uint8_t, 8> counts_;
  std::array<Point, 8> points_;

  Point origin_ = Point(0, 0, 0);  // 粗体素最小角（用于推算子体素中心）
  float sub_res_ = 0.0f;
};



template<typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  using KNNHeapType = KNNHeap<5, Point>;
  using OctVoxType = OctVox<Point>;

  struct Options {
    float resolution      = 0.5;   
    std::size_t capacity  = 1000000;

    Options(float __resolution, std::size_t __capacity) {
      resolution = __resolution;
      capacity = __capacity;
    }
  };

  

  OctVoxMap() {
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    reset_max_group();
  }
  
  ~OctVoxMap() {
    grids_.clear();
    data_.clear();
  }
  
  OctVoxMap(Options options){
    SetOptions(options);
    std::cout << " ---> OctVoxMap init. Resolution: " << resolution_ 
              << " Capacity: " << capacity_ << std::endl;
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    reset_max_group();
  }

  void SetOptions(const Options& options)
  {
    resolution_ = options.resolution;
    capacity_ = options.capacity;
    inv_resolution_ = 1.0 / resolution_;
    sub_resolution_ = resolution_ / 2.0;
    sub_inv_resolution_ = 1.0 / sub_resolution_;

    // M.29 尖峰治理：robin_map 不做预分配时按 2 倍序扩容，每次扩容要整表重哈希，
    // 在 10^5~10^6 体素量级上会偶发几十 ms 的尖峰（实测 1f 达 92 ms，是丢帧元凶）。
    // reserve 只改桶数量，不改键集/淘汰顺序 ⇒ 位姿输出应逐字节不变。
    if (g_map_hash_reserve_en) {
      const std::size_t want = (g_map_hash_reserve_size > 0)
                                 ? static_cast<std::size_t>(g_map_hash_reserve_size)
                                 : capacity_;
      grids_.reserve(want);
      std::cout << " ---> OctVoxMap hash reserve: " << want
                << " buckets=" << grids_.bucket_count() << std::endl;
    }
  }

  /// M.29 插入统计（周期性打印后清零）：新体素数 / 容量淘汰数 / 表规模与桶数
  struct InsertStats {
    std::size_t n_new   = 0;   // 本窗口新建体素数（触发 hash insert 的点）
    std::size_t n_evict = 0;   // 本窗口因容量上限被淘汰的体素数
  };
  InsertStats takeStats() { InsertStats s = stats_; stats_ = InsertStats{}; return s; }
  std::size_t mapSize()     const { return data_.size(); }
  std::size_t bucketCount() const { return grids_.bucket_count(); }

  void insert(const Points& cloud_world);
  void printInfo() const;
  void getMap(std::vector<float>&) const;
  void saveMap() const;    // TODO:
  void resetMap(const std::vector<float>&);
  void clear();

  void getTopK(const Point& point, KNNHeapType& top_K) const;

  void getTopK_VN(const Point& point, KNNHeapType& top_K) const;

  void reset_max_group(){
    const int all_groups = static_cast<int>(flat_search_order_offsets.size()) - 1;  // 6 组
    group_idx_max_ = (g_hknn_max_group < 0) ? all_groups
                                            : std::min(g_hknn_max_group, all_groups);
    if (group_idx_max_ < 1) group_idx_max_ = 1;
  }

  void decrease_max_group(){
    if(group_idx_max_ > 4) group_idx_max_--;
  }

  // size_t getMemoryUsageBytes() const {
  //   size_t bytes = 0;
  //   bytes += sizeof(*this);
  //   bytes += data_.size() * (sizeof(KEY) + sizeof(OctVoxType)
  //                           + sizeof(void*) * 2); // list node pointers
  //   bytes += grids_.size() * (sizeof(KEY) + sizeof(DATA_ITER)
  //                             + sizeof(size_t)); // hash & pair overhead
  //   bytes += grids_.bucket_count() * sizeof(void*); // bucket array
  //   bytes += flat_search_ptrs_.capacity() * sizeof(uint8_t*);
  //   return bytes;
  // }


private:
  float resolution_ = 0.5;
  float inv_resolution_ = 1.0;
  float sub_resolution_ = 0.25;
  float sub_inv_resolution_ = 4.0;
  std::size_t capacity_ = 1000000;

  bool reset_map_ = false;
  int reset_map_count_ = 0;

  const KEY nearby_grids_[19] = {
    KEY(0, 0, 0),
    KEY(-1, -1, 0), KEY(-1, 0, 0), KEY(-1, 1, 0), 
    KEY(0, -1, 0), KEY(0, 1, 0), 
    KEY(1, -1, 0), KEY(1, 0, 0), KEY(1, 1, 0), 
    KEY(0, 0, -1), KEY(1, 0, -1), KEY(-1, 0, -1), 
    KEY(0, 1, -1), KEY(0, -1, -1), 
    KEY(0, 0, 1), KEY(1, 0, 1), KEY(-1, 0, 1), 
    KEY(0, 1, 1), KEY(0, -1, 1)
  };

  /// HashShiftMix
  struct HASH_VEC {
    std::size_t operator()(const KEY &v) const {
      size_t h = static_cast<size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DATA_LIST = std::list<std::pair<KEY, OctVoxType>>;
  using DATA_ITER = typename DATA_LIST::iterator;

  DATA_LIST data_;
  tsl::robin_map<KEY, DATA_ITER, HASH_VEC> grids_;
  InsertStats stats_;

  std::vector<uint8_t*> flat_search_ptrs_;
  int group_idx_max_;

};


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::insert(const Points& cloud_world){
  if(reset_map_){
    reset_map_count_--;
    if(reset_map_count_ > 0){
      std::cout << "OctVoxMap::insert skip: reset_map_count_ = " << reset_map_count_ << std::endl;
      return;
    } 
    reset_map_ = false;
  }

  for(auto& pt : cloud_world){
    KEY fine_key = (pt * sub_inv_resolution_).array().floor().template cast<int>();
    KEY key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    uint8_t dx = fine_key[0] & 1;
    uint8_t dy = fine_key[1] & 1;
    uint8_t dz = fine_key[2] & 1;
    uint8_t local_idx = (dz << 2) | (dy << 1) | dx;

    auto iter = grids_.find(key);
    if (iter == grids_.end()) {
      const Point origin = key.cast<Scalar>() * resolution_;
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx, origin, sub_resolution_));
      grids_.insert(std::make_pair(key, data_.begin()));
      ++stats_.n_new;
      
      if (data_.size() >= capacity_) {
        grids_.erase(data_.back().first);
        data_.pop_back();
        ++stats_.n_evict;
      }
    } else {
      iter->second->second.AddPoint(pt, local_idx);
      data_.splice(data_.begin(), data_, iter->second);
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK(const Point& point, KNNHeapType& top_K) const {
  const KEY fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
  KEY key;
  key[0] = fine_key[0] >> 1;
  key[1] = fine_key[1] >> 1;
  key[2] = fine_key[2] >> 1;

  const int dx = fine_key[0] & 1;
  const int dy = fine_key[1] & 1;
  const int dz = fine_key[2] & 1;
  const int local_idx = (dz << 2) | (dy << 1) | dx;
  const KEY mirror_axis = KEY(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));
  
  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);
  
  for(uint8_t i = 0; i < pre_voxel_ptr_size; ++i)
  {
    KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[i]);
    KEY n_key = key + delta_key;
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      top_voxels_2_search[i] = &iter->second->second;
    }
  }

  Point __sub_point;

  for (int group_idx = 0; group_idx < group_idx_max_; ++group_idx) {
    const uint8_t* group_it = flat_search_ptrs_[group_idx];
    const uint8_t* group_end = flat_search_ptrs_[group_idx + 1];

    while(group_it < group_end){
      const uint8_t neighbor_idx = *group_it++;
      uint8_t data_size = *group_it++;
      
      if(neighbor_idx < pre_voxel_ptr_size)
      {
        OctVoxType* voxel_ptr = top_voxels_2_search[neighbor_idx];
        if (voxel_ptr) {
          while (data_size--) {
            uint8_t _local_idx = (*group_it++)^local_idx;
            if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
              const float dist2 = (__sub_point - point).squaredNorm();
              top_K.try_insert(dist2, __sub_point);
            }
          }
        }
        else group_it+=data_size;
        continue;
      }

      KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
      const KEY n_key = key + delta_key;

      if (auto iter = grids_.find(n_key); iter != grids_.end()){
        OctVoxType* voxel_ptr = &iter->second->second;
        while (data_size--){
          const uint8_t _local_idx = (*group_it++)^local_idx;
          if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
            float dist2 = (__sub_point - point).squaredNorm();
            top_K.try_insert(dist2, __sub_point);
          }
        }
      }
      else group_it+=data_size;
    }

    if (top_K.count == 5)
      if (top_K.max_dist2_ < orders_min_dis2[group_idx] * g_hknn_order_scale){
        break;
      }

  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK_VN(const Point& point, KNNHeapType& top_K) const{
  KEY key = (point * inv_resolution_).array().floor().template cast<int>();

  std::vector<OctVoxType*> voxels_2_search;
  voxels_2_search.reserve(19);
  for(std::size_t i = 0; i < 19; ++i) {
    KEY n_key = key + nearby_grids_[i];
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      voxels_2_search.emplace_back(&iter->second->second);
    }
  }

  Point pt;
  for(auto& voxel : voxels_2_search) {
    for(uint8_t _i = 0; _i < 8; ++_i) {
      if(!voxel->getPoint(_i, pt)) continue;
      float dist2 = (pt - point).squaredNorm();
      top_K.try_insert(dist2, pt);
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getMap(std::vector<float>& output) const{
  size_t total_points = 0;

  output.clear();
  output.reserve(total_points * 3);

  Point point;
  float pcl_point[3];
  for (const auto& voxel_pair : data_) {
    const OctVoxType& voxel = voxel_pair.second;
    for(uint8_t i = 0; i < 8; ++i) {
      if (!voxel.getPoint(i, point)) continue;
      pcl_point[0] = static_cast<float>(point.x());
      pcl_point[1] = static_cast<float>(point.y());
      pcl_point[2] = static_cast<float>(point.z());
      output.push_back(pcl_point[0]);
      output.push_back(pcl_point[1]);
      output.push_back(pcl_point[2]);
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::resetMap(const std::vector<float>& input){
  if (input.empty()) return;
  
  clear();
  size_t num_points = input.size() / 3;

  Points cloud_world;
  cloud_world.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    Point point(input[i * 3], input[i * 3 + 1], input[i * 3 + 2]);
    cloud_world.push_back(point);
  }

  insert(cloud_world);

  reset_map_ = true;
  reset_map_count_ = 10;
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::saveMap() const {

  const std::string g_root_dir = std::string(ROOT);
  std::string filename = g_root_dir + "map/OctVoxMap.pcd";

  if (std::filesystem::exists(filename)) {
    std::filesystem::remove(filename);
    std::cout << "Removed existing file: " << filename << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  size_t total_points = data_.size() * 8;
  
  cloud->points.reserve(total_points);
  
  for (const auto& voxel_pair : data_) {
    const OctVoxType& voxel = voxel_pair.second;
    for(uint8_t i = 0; i < 8; ++i) {
      pcl::PointXYZ pcl_point;
      Point point;
      if (!voxel.getPoint(i, point)) continue;
      pcl_point.x = static_cast<float>(point.x());
      pcl_point.y = static_cast<float>(point.y());
      pcl_point.z = static_cast<float>(point.z());
      cloud->points.push_back(pcl_point);
    }
  }
  
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  
  int result = pcl::io::savePCDFileBinary(filename, *cloud);
  
  if (result == 0) {
    std::cout << "Successfully saved " << cloud->points.size() 
              << " points to " << filename << " (binary format)" << std::endl;
  } else {
    std::cerr << "Error saving point cloud to " << filename << std::endl;
    throw std::runtime_error("Failed to save PCD file: " + filename);
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::clear() {
  grids_.clear();
  data_.clear();
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::printInfo() const {
    std::cout << " ---> OctVoxMap info. Size: " << data_.size() 
              << " Capacity: " << capacity_ << std::endl;
}

}

#endif