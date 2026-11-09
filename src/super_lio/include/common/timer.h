#ifndef TIMER_H_
#define TIMER_H_

#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <mutex>
#include <numeric>
#include <string>

/// 线程安全的运行时间 / 延迟统计计时器。
/// 写入方：process 线程（Evaluate）、IMU 输出线程 / 点云输出线程（Record）。
/// 读取方：周期报告定时器线程（PrintAndClear）、退出时（PrintAll）。
/// 所有方法都用内部锁保护，供并发读写使用。
class Timer {
 public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name_ = name;
      time_usage_in_ms_.emplace_back(time_usage);
    }
    std::string func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
    Record(func_name, time_used);
  }

  void Record(const std::string& func_name, double time_usage_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_usage_ms);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_usage_ms)});
    }
  }

  void PrintAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    LOG(INFO) << ">>> ===== Printing run time =====";
    for (const auto& r : records_) {
      const auto& vec = r.second.time_usage_in_ms_;
      if (vec.empty()) continue;
      double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
      double avg = sum / double(vec.size());
      double mx = *std::max_element(vec.begin(), vec.end());
      LOG(INFO) << "> [ " << r.first << " ] avg: " << avg
                << " ms , max: " << mx << " ms , calls: " << vec.size();
    }
  }

  /// 打印当前统计信息，然后清空记录，使下一次打印为独立窗口（用于周期报告）
  void PrintAndClear() {
    PrintAll();
    Clear();
  }

  /// clean the records
  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
  }

 private:
  std::mutex mutex_;
  std::map<std::string, TimerRecord> records_;
};

#endif
