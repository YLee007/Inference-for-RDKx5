#include "../include/vision_serial_driver/vision_serial_driver_node.hpp"
#include <atomic>
#include <thread>
#include <concurrentqueue.h>

serial_driver_node::serial_driver_node(std::string device_name, std::string node_name)
    : rclcpp::Node(node_name), 
      vArray{new visionArray}, 
      rArray{new robotArray},
      writeQueue_(1024),  // 初始化队列容量为1024
      dev_name{new std::string(device_name)},
      portConfig{new SerialPortConfig(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE)}, 
      ctx{IoContext(2)}
{
  RCLCPP_INFO(get_logger(), "节点:/%s启动", node_name.c_str());
  muzzleSpeedFilter.Size=10;
  
  // 内存清零
  memset(vArray->array, 0, sizeof(visionArray));
  memset(rArray->array, 0, sizeof(robotArray));

  // 启动异步发送线程（关键新增）
  writeThread_ = std::thread([this]() {
    constexpr size_t kBatchSize = 32;    // 每批次最大发送量
    constexpr auto kIdleSleep = 50us;    // 空闲休眠时间
    
    while (rclcpp::ok() && !stopWriteThread_.load()) {
      size_t sent = 0;
      visionArray data;

      // 批量发送模式
      while (sent < kBatchSize && writeQueue_.try_dequeue(data)) {
        try {
          serial_write(data.array, sizeof(data.array));
          sent++;
        } catch (const std::exception& e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "发送失败: %s", e.what());
        }
      }

      // 动态休眠控制
      std::this_thread::sleep_for(sent ? 1us : kIdleSleep);
    }
  });

  // 设置线程优先级（Linux系统）
  #ifdef __linux__
  struct sched_param param{.sched_priority = 90};
  if (pthread_setschedparam(writeThread_.native_handle(), SCHED_FIFO, &param)) {
    RCLCPP_WARN(get_logger(), "需要root权限设置线程优先级");
  }
  #endif

  // 原有定时器和话题初始化
  reopenTimer = create_wall_timer(1s, std::bind(&serial_driver_node::serial_reopen_callback, this));
  publishTimer = create_wall_timer(2ms, std::bind(&serial_driver_node::robot_callback, this));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  publisher = create_publisher<vision_interfaces::msg::Robot>("/serial_driver/robot", rclcpp::SensorDataQoS());
  autoAimSub = create_subscription<vision_interfaces::msg::AutoAim>(
      "/serial_driver/aim_target", rclcpp::SensorDataQoS(), 
      std::bind(&serial_driver_node::auto_aim_callback, this, std::placeholders::_1));

  // 启动串口读取线程
  serialReadThread = std::thread(&serial_driver_node::serial_read_thread, this);
  serialReadThread.detach();
}

serial_driver_node::~serial_driver_node()
{
  // 安全停止发送线程
  stopWriteThread_.store(true);
  if (writeThread_.joinable()) {
    writeThread_.join();
  }

  // 关闭串口
  if (serialDriver.port()->is_open()) {
    serialDriver.port()->close();
  }
}

void serial_driver_node::auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg)
{
  if (isOpen) {
    // 准备数据
    vArray->msg.head = 0xA5;
    vArray->msg.fire = vMsg.fire;
    vArray->msg.aimPitch = vMsg.aim_pitch;
    vArray->msg.aimYaw = vMsg.aim_yaw;
    vArray->msg.tracking = vMsg.tracking;

    // 智能入队策略（关键修改）
    if (writeQueue_.size_approx() > 800) {  // 80%容量时丢弃旧数据
      visionArray dummy;
      writeQueue_.try_dequeue(dummy);
      lostFrames_++;
    }

    if (!writeQueue_.enqueue(*vArray)) {     // 非阻塞入队
      lostFrames_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "队列溢出! 总丢帧: %lu", lostFrames_.load());
    }
  }
}

// 其余函数保持原有实现不变...

void serial_driver_node::serial_write(uint8_t *data, size_t len)
{
  std::vector<uint8_t> tempData(data, data + len);
  try
  {
    serialDriver.port()->send(tempData);
    // RCLCPP_INFO(get_logger(), "写入串口.");
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(get_logger(), "写入串口时发生错误.");
    isOpen = false;
  }
}

void serial_driver_node::auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg)
{
  if (isOpen)
  {
    vArray->msg.head = 0xA5;
    vArray->msg.fire = vMsg.fire;
    vArray->msg.aimPitch = vMsg.aim_pitch;
    vArray->msg.aimYaw = vMsg.aim_yaw;
    vArray->msg.tracking = vMsg.tracking;
    serial_write(vArray->array, sizeof(vArray->array));
  }
}

void serial_driver_node::robot_callback()
{
  if (isOpen)
  {
    try
    {
      auto msg = vision_interfaces::msg::Robot();
      msg.foe_color=rArray->msg.foeColor==1?1:0;
      msg.mode = rArray->msg.mode;
      msg.foe_color = rArray->msg.foeColor;
      msg.self_yaw = rArray->msg.robotYaw;
      msg.self_pitch = rArray->msg.robotPitch;
      double muzzle_speed = 15.0;
      muzzleSpeedFilter.get_avg(muzzle_speed);
      msg.muzzle_speed = muzzle_speed;
      publisher->publish(msg);

      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t.header.frame_id = "odom";
      t.child_frame_id = "gimbal_link";
      tf2::Quaternion q;
      q.setRPY(0, -rArray->msg.robotPitch*3.1415926535/180.0, rArray->msg.robotYaw*3.1415926535/180.0);
      t.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(t);

      if (!initial_set_param_ || msg.foe_color != previous_receive_color_) {
          
        auto param = rclcpp::Parameter("detect_color", msg.foe_color);

        if (!detector_param_client_->service_is_ready()) {
          RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
          return;
        }

        if (
          !set_param_future_.valid() ||
          set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
          set_param_future_ = detector_param_client_->set_parameters(
            {param}, [this, param](const ResultFuturePtr & results) {
              for (const auto & result : results.get()) {
                if (!result.successful) {
                  RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                  return;
                }
              }
              RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
              initial_set_param_ = true;
            });
        }
        previous_receive_color_ = msg.foe_color;
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 20, "处理串口数据时发生错误: %s", ex.what());
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::string dev_name = "/dev/ttyACM0";
  std::shared_ptr<serial_driver_node> node = std::make_shared<serial_driver_node>(dev_name, "vision_serial_driver");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}