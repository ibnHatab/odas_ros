
#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "loopback_stream.hpp"
#include "odas_json_parser.hpp"
#include "odas_ros_msgs/msg/odas_ssl.hpp"
#include "odas_ros_msgs/msg/odas_ssl_array_stamped.hpp"
#include "odas_ros_msgs/msg/odas_sst.hpp"
#include "odas_ros_msgs/msg/odas_sst_array_stamped.hpp"
#include "std_msgs/msg/header.hpp"

// From odas demo client
extern "C" {
#include <configs.h>
#include <objects.h>
#include <odas.h>
#include <parameters.h>
#include <profiler.h>
#include <threads.h>
}

using json = nlohmann::json;

class OdasNode : public rclcpp::Node {
  const int SSL_PORT = 9001;
  const int SST_PORT = 9000;

 public:
  OdasNode() : Node("odas_node") {
    // Create a publisher for OdasSstArray messages
    ssl_publisher_ = this->create_publisher<odas_ros_msgs::msg::OdasSslArrayStamped>("ssl", 10);
    auto ssl_callback = [this](const odas_ros_msgs::msg::OdasSslArrayStamped& msg) {
      ssl_publisher_->publish(msg);
    };
    sst_publisher_ = this->create_publisher<odas_ros_msgs::msg::OdasSstArrayStamped>("sst", 10);
    auto sst_callback = [this](const odas_ros_msgs::msg::OdasSstArrayStamped& msg) {
      sst_publisher_->publish(msg);
    };

    auto ros_error = [this](const std::string& msg) {
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
    };

    ssl_parser_ = std::make_unique<SslEventConsumer>(ssl_callback, ros_error);
    auto ssl_input = [this](const std::string& text) {
      json::sax_parse(text, this->ssl_parser_.get());
    };
    ssl_socket_ = std::make_unique<LoopbackStream>(ssl_input, SSL_PORT, ros_error);

    sst_parser_ = std::make_unique<SstEventConsumer>(sst_callback, ros_error);
    auto sst_input = [this](const std::string& text) {
      json::sax_parse(text, this->sst_parser_.get());
    };
    sst_socket_ = std::make_unique<LoopbackStream>(sst_input, SST_PORT, ros_error);

    // Set up a timer to read data from the socket
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
      ssl_socket_->read_socket();
      sst_socket_->read_socket();
    });

    std::string configFile = declare_parameter("configuration_path", "");
    RCLCPP_INFO(get_logger(), "Using configuration file = %s", configFile.c_str());

    RCLCPP_INFO(get_logger(), "| + Initializing configurations...... ");
    cfgs = configs_construct(configFile.c_str());
    RCLCPP_INFO(get_logger(), "| + Initializing objects............. ");
    aobjs = aobjects_construct(cfgs);
    RCLCPP_INFO(get_logger(), "| + Launch threads................... ");
    threads_multiple_start(aobjs);

    RCLCPP_INFO(get_logger(), "| + Accept socket................... ");
    ssl_socket_->accept_socket();
    sst_socket_->accept_socket();

    RCLCPP_INFO(get_logger(), "| + ROS SPINNING................... ");
  }

  ~OdasNode() {
    ssl_socket_.reset();
    sst_socket_.reset();

    RCLCPP_INFO(get_logger(), "| + Threads join.................. ");
    threads_multiple_join(aobjs);
    RCLCPP_INFO(get_logger(), "| + Free memory...................... ");

    aobjects_destroy(aobjs);
    configs_destroy(cfgs);
    RCLCPP_INFO(get_logger(), "Done!");
  }

 private:
  rclcpp::Publisher<odas_ros_msgs::msg::OdasSslArrayStamped>::SharedPtr ssl_publisher_;
  rclcpp::Publisher<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<SslEventConsumer> ssl_parser_;
  std::unique_ptr<LoopbackStream> ssl_socket_;

  std::unique_ptr<SstEventConsumer> sst_parser_;
  std::unique_ptr<LoopbackStream> sst_socket_;

  aobjects* aobjs = NULL;
  configs* cfgs = NULL;
};

// Publish the message

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<OdasNode> node = std::make_shared<OdasNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
