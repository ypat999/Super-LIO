
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
    // M.35：邻域大小 K 提到运行时（模板参数仍决定数组容量，故这里夹到容量内，
    // 传错值最多退化到"按容量运行"，不会越界）。下限由参数装载处保证 ≥4。
    const uint8_t kmax = static_cast<uint8_t>(eff_k());
    const bool not_full = (count < kmax);
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
  /// 求堆内最坏（距离最大）项的下标与距离。
  /// 历史缺陷（M.35 修复）：本函数原先手写 unrolled 只扫 dist2_[0..4]，
  /// 而 67c0acd 把容量从 5 提到 8 时没有同步——满堆后 worst_/max_dist2_
  /// 永远只在前 5 个位置里取最大，于是：
  ///   1) 索引 5..7 里的远点永远不会被替换掉，堆不再维护"最近 K 个"；
  ///   2) max_dist2_ 被系统性低估，try_insert 的 dist2<max_dist2_ 判据变严，
  ///      getTopK 的提前退出判据也跟着错；
  ///   3) 最终 top_K.count 可达 8，把上面这些不该用的点送进平面拟合。
  /// 这里改成扫描全部 K 个位置：K 是模板常量，编译器会完全展开，
  /// 与被屏蔽的尾部位置 dist2_==0 不影响结果（平方距离非负）。
  /// K=5 时与修复前的 unrolled 版逐位等价（有效点全在前 5 个位置）。
  inline void update_worst_unrolled() {
    float mx = dist2_[0];
    uint8_t wi = 0;
    for (int i = 1; i < K; ++i) {
      if (dist2_[i] > mx) { mx = dist2_[i]; wi = static_cast<uint8_t>(i); }
    }
    worst_ = wi;
    max_dist2_ = mx;
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
  static constexpr uint8_t kMaxSlots = 64;  // sub_div=4 时 4×4×4

  OctVox(const Point& pt, uint8_t local_idx, const Point& origin = Point(0, 0, 0),
         float sub_res = 0.0f, uint8_t axis_bits = 1)
      : origin_(origin), sub_res_(sub_res), axis_bits_(axis_bits)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  /// 子体素几何中心（由粗体素原点 + local_idx 推算）。
  /// axis_bits=1 → 2×2×2（idx 位 [a][b][c]）；axis_bits=2 → 4×4×4
  /// （idx = dx | dy<<2 | dz<<4，dx∈0..3）。两种编码下 SubCenter 统一为
  /// (2*axis+1)*half，half = sub_res/2，axis=1 时与旧版 (idx&1?3:1)*half 等价。
  Point SubCenter(uint8_t local_idx) const {
    const float half = 0.5f * sub_res_;
    const uint8_t mask = (1u << axis_bits_) - 1u;
    const uint8_t dx = local_idx & mask;
    const uint8_t dy = (local_idx >> axis_bits_) & mask;
    const uint8_t dz = (local_idx >> (2u * axis_bits_)) & mask;
    return Point(origin_.x() + (2.0f * dx + 1.0f) * half,
                 origin_.y() + (2.0f * dy + 1.0f) * half,
                 origin_.z() + (2.0f * dz + 1.0f) * half);
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

  // 固定按 sub_div=4 的上限开槽：sub_div=2 只用前 8 槽（idx 0..7 连续，
  // 冷槽不触碰热路径缓存），亚字节级行为与旧版一致。
  std::array<uint8_t, kMaxSlots> counts_;
  std::array<Point, kMaxSlots> points_;

  Point origin_ = Point(0, 0, 0);  // 粗体素最小角（用于推算子体素中心）
  float sub_res_ = 0.0f;
  uint8_t axis_bits_ = 1;          // 每轴子体素位宽：1 → 2 分/轴，2 → 4 分/轴
};



template<typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  // 实验：平面拟合邻域 K=5 → 8（与官方 FAST-LIO2 的 5 邻域对齐后再放大，
  // 用更多点抑制均值压缩带来的参考面抖动）。KNNHeapType 与下方 getTopK
  // 的提前退出判断必须同步改，否则堆会静默只填 5 个点。
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

  /// sub_div=4 专用搜索：邻居粗体素全槽扫描（无子槽预计算表），
  /// 早退阈值按粗体素尺寸缩放。
  void getTopK_sub4(const Point& point, KNNHeapType& top_K) const;

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
  uint8_t sub_bits_ = 1;      // 每轴位宽（sub_div=2→1，4→2）
  int sub_mask_ = 1;          // (1<<sub_bits_)-1
  uint8_t sub_slots_ = 8;     // sub_div³
  float hknn_thr_scale_ = 1.0f;
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
      const Point origin = key.cast<Scalar>() * resolution_;
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx, origin, sub_resolution_, sub_bits_));
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
      if (top_K.max_dist2_ < orders_min_dis2[group_idx] * g_hknn_order_scale){
        break;
      }

  }
}


/// sub_div=4（4×4×4=64 槽）专用搜索。flat_search_order 的子槽候选表是为
/// 2×2×2 编码手工生成的，无法复用到 2bit/轴 的编码上；这里保留其"邻居粗体素
/// 由近到远分组 + 早退阈值"的结构，但把每组内的子槽查找退化为整槽扫描：
/// 组边界仍按 neighbor_idx 分组遍历（丢弃 data_size 之后的子槽列表），
/// 命中邻居体素后扫描其全部 sub_slots_ 个槽位。
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

  // 镜像轴：查询点位于粗体素上半（dx>=2）时，邻居偏移与扫描方向整体翻转，
  // 使预计算表的近邻顺序仍从查询点一侧展开（对应 2bit 版的 3-dx = dx^3）。
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
      group_it += data_size;  // 子槽候选表对 4 分/轴编码无效，跳过并全槽扫描

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
      if (top_K.max_dist2_ < orders_min_dis2[group_idx] * g_hknn_order_scale * hknn_thr_scale_) {
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