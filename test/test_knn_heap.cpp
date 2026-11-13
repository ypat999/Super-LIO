// KNNHeap 正确性测试：与暴力 Top-K 结果逐项对照。
//
// 编译 & 运行：
//   g++ -std=c++17 -O2 -I src/super_lio/include -I /usr/include/eigen3 \
//       test/test_knn_heap.cpp -o /tmp/test_knn_heap && /tmp/test_knn_heap
//
// 判据：返回的邻居集合（距离多重集 + 点集）必须与暴力排序后的精确 Top-K 完全一致，
// 而不是看条件数/残差这类下游指标。

#include <algorithm>
#include <array>
#include <cstdio>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "OctVoxMap/KNNHeap.hpp"

using LI2Sup::KNNHeap;

namespace {

struct Item {
  float d2;
  float x, y, z;
};

// 暴力参考：按 dist2 升序取前 K 个
std::vector<Item> brute_topk(std::vector<Item> items, int k) {
  std::sort(items.begin(), items.end(),
            [](const Item& a, const Item& b) { return a.d2 < b.d2; });
  if (static_cast<int>(items.size()) > k) items.resize(k);
  return items;
}

bool same_xyz(const std::array<float, 3>& a, const std::array<float, 3>& b) {
  if (a[0] != b[0]) return a[0] < b[0];
  if (a[1] != b[1]) return a[1] < b[1];
  return a[2] < b[2];
}

// compare_points=false 时只比距离多重集（存在等距候选时，保留哪一个点是任意的）
template <int K>
bool check_case(const char* name, const std::vector<Item>& items,
                bool compare_points = true) {
  KNNHeap<K, Eigen::Vector3f> heap;
  for (const auto& it : items) {
    heap.try_insert(it.d2, Eigen::Vector3f(it.x, it.y, it.z));
  }

  const int expect_n = std::min<int>(K, static_cast<int>(items.size()));
  if (heap.count != expect_n) {
    printf("[FAIL] %-34s count=%d expect=%d\n", name, heap.count, expect_n);
    return false;
  }

  const auto ref = brute_topk(items, K);
  bool ok = true;

  std::vector<float> got(heap.dist2_, heap.dist2_ + heap.count);
  std::vector<float> exp;
  exp.reserve(ref.size());
  for (const auto& r : ref) exp.push_back(r.d2);
  std::sort(got.begin(), got.end());
  std::sort(exp.begin(), exp.end());
  if (got != exp) {
    printf("[FAIL] %-34s dist multiset mismatch\n  got:", name);
    for (float d : got) printf(" %.9g", d);
    printf("\n  exp:");
    for (float d : exp) printf(" %.9g", d);
    printf("\n");
    ok = false;
  }

  if (ok && compare_points) {
    std::vector<std::array<float, 3>> gp, ep;
    for (int i = 0; i < heap.count; ++i)
      gp.push_back(std::array<float, 3>{heap.points_[i].x(), heap.points_[i].y(),
                                        heap.points_[i].z()});
    for (const auto& r : ref) ep.push_back(std::array<float, 3>{r.x, r.y, r.z});
    std::sort(gp.begin(), gp.end(), same_xyz);
    std::sort(ep.begin(), ep.end(), same_xyz);
    if (gp != ep) {
      printf("[FAIL] %-34s point set mismatch\n", name);
      ok = false;
    }
  }

  // 堆满时 max_dist2_ / worst_ 必须指向槽位内的真实最大值
  if (ok && heap.count == K) {
    float m = 0.0f;
    for (int i = 0; i < K; ++i) m = std::max(m, heap.dist2_[i]);
    if (heap.max_dist2_ != m) {
      printf("[FAIL] %-34s max_dist2_=%g expect=%g\n", name, heap.max_dist2_, m);
      ok = false;
    } else if (heap.dist2_[heap.worst_] != m) {
      printf("[FAIL] %-34s worst_=%d dist=%g expect max=%g\n", name, heap.worst_,
             heap.dist2_[heap.worst_], m);
      ok = false;
    }
  }

  if (ok) printf("[ ok ] %-34s (n=%zu)\n", name, items.size());
  return ok;
}

}  // namespace

int main() {
  bool all_ok = true;

  // 1) 候选数 < / == / > K
  {
    std::vector<Item> v;
    for (int i = 0; i < 3; ++i) v.push_back({float(i + 1), float(i), 0.f, 0.f});
    all_ok &= check_case<8>("n<K (3)", v);
  }
  {
    std::vector<Item> v;
    for (int i = 0; i < 8; ++i) v.push_back({float(i + 1), float(i), 0.f, 0.f});
    all_ok &= check_case<8>("n==K (8)", v);
  }
  {
    std::vector<Item> v;
    for (int i = 0; i < 20; ++i) v.push_back({float(i + 1), float(i), 0.f, 0.f});
    all_ok &= check_case<8>("n>K 单调递增", v);
  }

  // 2) 旧 bug 直接回归：先让 8 个远点占满堆（第 6~8 槽最远），再逐个插入更近的点。
  //    旧实现只在 dist2_[0..4] 里找最远，会永远淘汰不掉第 6~8 槽的远点。
  {
    std::vector<Item> v;
    for (int i = 0; i < 8; ++i) v.push_back({float(100 + i), float(i), 0.f, 0.f});
    for (int i = 0; i < 8; ++i) v.push_back({float(1 + i), float(i), 0.f, 0.f});
    all_ok &= check_case<8>("远点占满后逐个替换", v);
    std::reverse(v.begin(), v.end());
    all_ok &= check_case<8>("远点占满后逐个替换(逆序)", v);
  }

  // 3) 随机点集 + 随机插入顺序，多种子、多种候选数；同时跑 K=5 防止改坏旧行为
  {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> uni(-50.f, 50.f);
    for (int trial = 0; trial < 200; ++trial) {
      const int n = 1 + (trial * 7) % 64;
      std::vector<Item> v;
      v.reserve(n);
      for (int i = 0; i < n; ++i) {
        const float x = uni(rng), y = uni(rng), z = uni(rng);
        v.push_back({x * x + y * y + z * z, x, y, z});
      }
      std::shuffle(v.begin(), v.end(), rng);
      char name[96];
      snprintf(name, sizeof(name), "random K=8 n=%d trial=%d", n, trial);
      all_ok &= check_case<8>(name, v);
      snprintf(name, sizeof(name), "random K=5 n=%d trial=%d", n, trial);
      all_ok &= check_case<5>(name, v);
    }
  }

  // 4) 距离相等 / 极度接近：只比距离多重集
  {
    std::vector<Item> v;
    for (int i = 0; i < 16; ++i) v.push_back({1.0f, float(i), 0.f, 0.f});
    all_ok &= check_case<8>("全部等距", v, false);

    v.clear();
    for (int i = 0; i < 16; ++i)
      v.push_back({1.0f + (i % 2) * 1e-7f, float(i), 0.f, 0.f});
    all_ok &= check_case<8>("接近等距", v, false);
  }

  printf(all_ok ? "\nALL PASS\n" : "\nSOME FAILED\n");
  return all_ok ? 0 : 1;
}
