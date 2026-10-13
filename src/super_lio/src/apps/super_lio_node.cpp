

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"

#include <signal.h>

void sigterm_handler(int signum)
{
  rclcpp::shutdown();
}


using namespace LI2Sup;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  signal(SIGTERM, sigterm_handler);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  data_wrapper->setSuperLIO(lio);
  lio->init();

  rclcpp::on_shutdown([lio]() {
    lio->printTimeRecord();
  });

  // process 定时器使用独立回调组，与 IMU/lidar 回调并行
  auto timer = data_wrapper->create_wall_timer(
    std::chrono::milliseconds(2),
    [lio]() { lio->process(); },
    data_wrapper->getProcessCallbackGroup()
  );

  // 性能统计定时器：lio.eva.timer 开启时每分钟输出一次并清零窗口；关闭时为空操作
  auto report_timer = data_wrapper->create_wall_timer(
    std::chrono::seconds(60),
    [lio]() { lio->periodicTimeReport(); },
    data_wrapper->getReportCallbackGroup()
  );

  // 多线程执行器：4 线程 = IMU 线程 + Lidar 线程 + process 线程 + 周期报告
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(data_wrapper);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
