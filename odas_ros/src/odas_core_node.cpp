#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

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
 public:
  OdasNode() : Node("socket_node"), server_fd_(-1), client_fd_(-1) {
    // Create a publisher for OdasSstArray messages
    publisher_ =
        this->create_publisher<odas_ros_msgs::msg::OdasSstArrayStamped>("odas_sources", 10);

    // Set up the socket
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket creation error");
      rclcpp::shutdown();
      return;
    }

    // Assign the socket address
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(PORT);

    // Bind the socket
    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed");
      rclcpp::shutdown();
      return;
    }

    // Start listening on the socket
    if (listen(server_fd_, 3) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Listen failed");
      rclcpp::shutdown();
      return;
    }

    // Set up a timer to read data from the socket
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&OdasNode::read_socket, this));

    std::string configFile = declare_parameter("configuration_path", "");
    RCLCPP_INFO(get_logger(), "Using configuration file = %s", configFile.c_str());

    RCLCPP_INFO(get_logger(), "| + Initializing configurations...... ");
    cfgs = configs_construct(configFile.c_str());
    RCLCPP_INFO(get_logger(), "| + Initializing objects............. ");
    aobjs = aobjects_construct(cfgs);
    RCLCPP_INFO(get_logger(), "| + Launch threads................... ");
    threads_multiple_start(aobjs);

    RCLCPP_INFO(get_logger(), "| + Accept socket................... ");
    accept_socket();

    RCLCPP_INFO(get_logger(), "| + ROS SPINNING................... ");
  }

  ~OdasNode() {
    if (server_fd_ != -1) {
      close(server_fd_);
    }
    if (client_fd_ != -1) {
      close(client_fd_);
    }

    RCLCPP_INFO(get_logger(), "| + Threads join.................. ");
    threads_multiple_join(aobjs);
    RCLCPP_INFO(get_logger(), "| + Free memory...................... ");

    aobjects_destroy(aobjs);
    configs_destroy(cfgs);
    RCLCPP_INFO(get_logger(), "Done!");
  }

 private:
  void read_socket();
  void accept_socket();
  void handle_data(const json& data);

  rclcpp::Publisher<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int server_fd_, client_fd_;
  struct sockaddr_in address_;
  int addrlen_ = sizeof(address_);
  const int PORT = 9000;
  std::string json_data_;  // To hold incoming JSON data

  aobjects* aobjs = NULL;
  configs* cfgs = NULL;
};

void OdasNode::accept_socket() {
  // Accept the connection
  if ((client_fd_ = accept(server_fd_, (struct sockaddr*)&address_, (socklen_t*)&addrlen_)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Accept failed");
    rclcpp::shutdown();
    return;
  }
}

void OdasNode::handle_data(const json& data) {
  // Extract data from JSON and publish
  odas_ros_msgs::msg::OdasSstArrayStamped odas_sst_array;
  odas_sst_array.header.stamp = this->now();
  odas_sst_array.header.frame_id = "frame_id";  // Set your frame ID accordingly

  for (const auto& source : data["src"]) {
    if (source["id"] != 0) {
      odas_ros_msgs::msg::OdasSst odas_sst;
      odas_sst.id = source["id"];
      odas_sst.x = source["x"];
      odas_sst.y = source["y"];
      odas_sst.z = source["z"];
      odas_sst.activity = source["activity"];
      odas_sst_array.sources.push_back(odas_sst);
    }
  }

  // Publish the message
  publisher_->publish(odas_sst_array);
}

std::vector<std::string> split_json(const std::string& data, size_t& consumed) {
  std::vector<std::string> messages;
  std::string json_buffer;
  int depth = 0;
  size_t json_start_index = 0;

  for (size_t i = 0; i < data.size(); ++i) {
    if (data[i] == '{') {
      if (depth == 0) {
        json_start_index = i;
      }
      depth++;
    } else if (data[i] == '}') {
      depth--;
      if (depth == 0) {
        messages.push_back(data.substr(json_start_index, i - json_start_index + 1));
        consumed = i;
      }
    }
  }

  return messages;
}

void OdasNode::read_socket() {
  const int buffer_size = 512;
  char buffer[buffer_size] = {0};

  ssize_t bytes_read = read(client_fd_, buffer, sizeof(buffer) - 1);

  if (bytes_read > 0) {
    buffer[bytes_read] = '\0';  // Null-terminate the string
    std::cout << "0 >>" << bytes_read << std::endl;

    std::string data(buffer);
    json_data_.append(data);

    RCLCPP_ERROR(this->get_logger(), "1 >> ");

    RCLCPP_ERROR(this->get_logger(), data.c_str());
    RCLCPP_ERROR(this->get_logger(), "2 >> ");
    RCLCPP_ERROR(this->get_logger(), json_data_.c_str());

    try {
      // Parse the JSON data
      size_t consumed = 0;
      auto messages = split_json(json_data_, consumed);
      std::cout << "0.1 >>" << consumed << std::endl;
      json_data_.erase(0, consumed);

      for (const auto& message : messages) {
        RCLCPP_ERROR(this->get_logger(), "2 >> ");
        RCLCPP_ERROR(this->get_logger(), message.c_str());
        json parsed_json = json::parse(message);
        handle_data(parsed_json);
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error parsing JSON: %s", e.what());
    }
  } else if (bytes_read == -1) {
    RCLCPP_ERROR(get_logger(), "Error reading from socket");
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<OdasNode> node = std::make_shared<OdasNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
