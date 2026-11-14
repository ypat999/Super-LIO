
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
    // 邻域大小 K 提到运行时（g_plane_knn_k）：模板参数仍决定数组容量，
    // 这里夹到容量内，参数传错最多退化为"按容量运行"，不会越界。
    const bool not_full = (count < eff_k());
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
        update_worst();
      }
    }
  }

private:
  inline void update_worst() {
    uint8_t worst = 0;
    float max_d = dist2_[0];
    for (int i = 1; i < K; ++i) {
      if (dist2_[i] > max_d) {
        max_d = dist2_[i];
        worst = static_cast<uint8_t>(i);
      }
    }
    worst_ = worst;
    max_dist2_ = max_d;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
  static constexpr int capacity() { return K; }
  /// 运行时生效的邻域大小（受参数 g_plane_knn_k 与模板容量 K 共同决定）
  inline int eff_k() const { return std::min(g_plane_knn_k, K); }
};


template<typename Point>
class OctVox{
public:
  /// 槽位上限按 sub_div=4（4×4×4=64）开；sub_div=2 只用前 8 槽，
  /// 旧编码 idx = dz<<2 | dy<<1 | dx 落在 0..7，与旧版逐位一致。
  static constexpr uint8_t kMaxSlots = 64;

  OctVox(const Point& pt, uint8_t local_idx)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  void AddPoint(const Point& pt, uint8_t local_idx) {
    uint8_t& count = counts_[local_idx];
    Point& stored_point = points_[local_idx];
    if(count == UNINIT_MASK) {
      stored_point = pt;
      count = 1;
      return;
    }

    if(count >= MAX_POINTS_PER_SUBVOXEL) return;
    if ((pt - stored_point).squaredNorm() > DISTANCE_THRESHOLD_SQ) return;

    stored_point = (stored_point * count + pt) / (count + 1);
    ++count;
  }

  bool getPoint(const uint8_t local_idx, Point& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  static constexpr uint8_t UNINIT_MASK = 0x00;
  static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL = 20;
  static constexpr double DISTANCE_THRESHOLD_SQ = 0.1 * 0.1;

  std::array<uint8_t, kMaxSlots> counts_;
  std::array<Point, kMaxSlots> points_;
};



template<typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  using KNNHeapType = KNNHeap<8, Point>;
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
    group_idx_max_ = flat_search_order_offsets.size() - 1;
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
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  void SetOptions(const Options& options)
  {
    resolution_ = options.resolution;
    capacity_ = options.capacity;
    inv_resolution_ = 1.0 / resolution_;

    // 子体素细分因子 sub_div：2 = 旧行为（2×2×2=8 槽，HKNN 预计算子槽候选表可用）；
    // 4 = 4×4×4=64 槽（点位粒度 = 粗体素/4，搜索退化为按邻居粗体素全槽扫描，
    // 早退阈值按 (resolution_/0.5)² 折算——orders_min_dis2 是按 res=0.5 标的米制常数）。
    const uint8_t sub_div = (g_map_sub_div == 4) ? 4 : 2;
    sub_bits_ = (sub_div == 4) ? 2 : 1;
    sub_mask_ = (1u << sub_bits_) - 1u;
    sub_slots_ = static_cast<uint8_t>(sub_div * sub_div * sub_div);
    hknn_thr_scale_ = (sub_div == 4)
                        ? (resolution_ / 0.5f) * (resolution_ / 0.5f)
                        : 1.0f;

    sub_resolution_ = resolution_ / static_cast<float>(sub_div);
    sub_inv_resolution_ = 1.0 / sub_resolution_;
    std::cout << " ---> OctVoxMap sub_div=" << static_cast<int>(sub_div)
              << " sub_resolution=" << sub_resolution_
              << " thr_scale=" << hknn_thr_scale_ << std::endl;

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

  /// sub_div=4 专用搜索：邻居粗体素全槽扫描（子槽预计算表按 2×2×2 编码生成，
  /// 无法复用到 2bit/轴，故按组丢弃子槽列表、整槽扫描），早退阈值按粗尺寸缩放。
  void getTopK_sub4(const Point& point, KNNHeapType& top_K) const;

  void getTopK_VN(const Point& point, KNNHeapType& top_K) const;

  void reset_max_group(){
    group_idx_max_ = flat_search_order_offsets.size() - 1;
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
  uint8_t sub_bits_ = 1;       // 每轴位宽（sub_div=2→1，4→2）
  int sub_mask_ = 1;           // (1<<sub_bits_)-1
  uint8_t sub_slots_ = 8;      // sub_div³
  float hknn_thr_scale_ = 1.0f;  // <1 时提前终止阈值按粗体素尺寸折算
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
    key[0] = fine_key[0] >> sub_bits_;
    key[1] = fine_key[1] >> sub_bits_;
    key[2] = fine_key[2] >> sub_bits_;

    uint8_t dx = fine_key[0] & sub_mask_;
    uint8_t dy = fine_key[1] & sub_mask_;
    uint8_t dz = fine_key[2] & sub_mask_;
    uint8_t local_idx = (dz << (2u * sub_bits_)) | (dy << sub_bits_) | dx;

    auto iter = grids_.find(key);
    if (iter == grids_.end()) {
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx));
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
  if (sub_bits_ != 1) { getTopK_sub4(point, top_K); return; }

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

    if (top_K.count == static_cast<uint8_t>(top_K.eff_k()))
      if (top_K.max_dist2_ < orders_min_dis2[group_idx]){
        break;
      }

  }
}


/// sub_div=4：组内子槽候选表按 2bit/轴 → 3bit/轴 编码，无法异或复用，
/// 故保留"由近到远分组 + 早退阈值"结构，但每组退化为整槽扫描（共 sub_slots_ 槽）。
template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK_sub4(const Point& point, KNNHeapType& top_K) const {
  const KEY fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
  KEY key;
  key[0] = fine_key[0] >> sub_bits_;
  key[1] = fine_key[1] >> sub_bits_;
  key[2] = fine_key[2] >> sub_bits_;

  const int dx = fine_key[0] & sub_mask_;
  const int dy = fine_key[1] & sub_mask_;
  const int dz = fine_key[2] & sub_mask_;

  // 查询点落在粗体素后半（dx >= 2）时，邻居展开方向整体翻转，
  // 使预计算表的由近到远顺序仍从查询点一侧开始。
  const int half_div = 1 << (sub_bits_ - 1);
  const KEY mirror_axis = KEY(dx >= half_div ? -1 : 1,
                              dy >= half_div ? -1 : 1,
                              dz >= half_div ? -1 : 1);

  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);

  for (uint8_t i = 0; i < pre_voxel_ptr_size; ++i) {
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

    while (group_it < group_end) {
      const uint8_t neighbor_idx = *group_it++;
      const uint8_t data_size = *group_it++;
      group_it += data_size;  // 子槽候选表对 2bit/轴 编码无效，跳过并整槽扫描

      OctVoxType* voxel_ptr = nullptr;
      if (neighbor_idx < pre_voxel_ptr_size) {
        voxel_ptr = top_voxels_2_search[neighbor_idx];
      } else {
        KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
        KEY n_key = key + delta_key;
        if (auto iter = grids_.find(n_key); iter != grids_.end())
          voxel_ptr = &iter->second->second;
      }
      if (!voxel_ptr) continue;

      for (uint8_t _i = 0; _i < sub_slots_; ++_i) {
        if (voxel_ptr->getPoint(_i, __sub_point)) {
          const float dist2 = (__sub_point - point).squaredNorm();
          top_K.try_insert(dist2, __sub_point);
        }
      }
    }

    if (top_K.count == static_cast<uint8_t>(top_K.eff_k()))
      if (top_K.max_dist2_ < orders_min_dis2[group_idx] * hknn_thr_scale_) {
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
    for(uint8_t _i = 0; _i < sub_slots_; ++_i) {
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
    for(uint8_t i = 0; i < sub_slots_; ++i) {
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
  size_t total_points = data_.size() * sub_slots_;
  
  cloud->points.reserve(total_points);
  
  for (const auto& voxel_pair : data_) {
    const OctVoxType& voxel = voxel_pair.second;
    for(uint8_t i = 0; i < sub_slots_; ++i) {
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