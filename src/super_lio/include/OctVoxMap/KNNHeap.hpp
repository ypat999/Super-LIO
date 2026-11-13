/**
 * @file KNNHeap.hpp
 * @brief 定长 K 近邻堆（无堆分配）。
 *
 * 从 OctVoxMap.hpp 抽出，便于单独做单元测试（test/test_knn_heap.cpp）。
 * 语义：
 *  - 未填满时按到达顺序占用槽位，并维护当前最远槽位 worst_ / max_dist2_；
 *  - 填满后，新候选必须严格小于 max_dist2_ 才会替换最远槽位；
 *  - 最远槽位的定位必须遍历全部 K 个槽位，否则 K>5 时会漏掉后面的远点，
 *    返回的邻居集合不再是全局最近的 K 个。
 */

#ifndef KNN_HEAP_HPP_
#define KNN_HEAP_HPP_

#include <array>
#include <cstdint>
#include <cstring>

namespace LI2Sup {

template <int K, typename Point>
class KNNHeap {
public:
  /// 堆容量（槽位数），供外部做 count == kSize 之类的判断，避免硬编码
  static constexpr int kSize = K;

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
        update_worst();
      }
    }
  }

private:
  /// 堆已填满时重新定位最远元素（遍历全部 K 个槽位）
  inline void update_worst() {
    uint8_t worst = 0;
    float max_dist2 = dist2_[0];
    for (int i = 1; i < K; ++i) {
      if (dist2_[i] > max_dist2) {
        max_dist2 = dist2_[i];
        worst = static_cast<uint8_t>(i);
      }
    }
    worst_ = worst;
    max_dist2_ = max_dist2;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
};

}  // namespace LI2Sup

#endif  // KNN_HEAP_HPP_
