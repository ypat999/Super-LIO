

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

  // 多线程执行器：3 线程 = IMU 线程 + Lidar 线程 + process 线程
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(data_wrapper);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
