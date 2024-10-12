#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include "odas_ros_msgs/msg/odas_ssl.hpp"
#include "odas_ros_msgs/msg/odas_ssl_array_stamped.hpp"
#include "odas_ros_msgs/msg/odas_sst.hpp"
#include "odas_ros_msgs/msg/odas_sst_array_stamped.hpp"

using json = nlohmann::json;
static const std::string frame_id = "odas";

template <typename ArrayType, typename SrcType>
class EventConsumer : public json::json_sax_t {
 public:
  using OdasArrayCallback = std::function<void(const ArrayType&)>;
  using ErrorCallback = std::function<void(const std::string& msg)>;

  EventConsumer(OdasArrayCallback cb, ErrorCallback on_error) : callback(cb), error(on_error) {}

  bool start_object(std::size_t elements) override {
    (void)elements;
    if (!inside_src) {
      odas_array = ArrayType();
    }
    return true;
  }

  bool end_object() override {
    // Add src_data to odas_sst_array.src when we finish reading a src object
    if (inside_src) {
      odas_array.sources.push_back(src_data);
      return true;
    } else {
      odas_array.header.frame_id = frame_id;
      odas_array.header.stamp = rclcpp::Clock().now();
      
      callback(odas_array);
      return true;
    }
    return true;
  }

  bool start_array(std::size_t elements) override {
    (void)elements;
    if (current_key == "src") {
      inside_src = true;
    }
    return true;
  }

  bool end_array() override {
    if (inside_src) {
      inside_src = false;  // Finished reading "src" array
    }
    return true;
  }

  bool key(string_t& val) override {
    current_key = val;
    return true;
  }

  bool number_unsigned(number_unsigned_t val) override {
    if (current_key == "timeStamp") {
      odas_array.odas_time_stamp = static_cast<int>(val);
    }
    return true;
  }

  bool parse_error(std::size_t position, const std::string& last_token,
                   const json::exception& ex) override {
    if (last_token.back() == '{') {
      throw std::make_pair(position, last_token);
    } else {
      std::string msg = "parse_error(position=" + std::to_string(position) +
                        ", last_token=" + last_token + ", ex=" + std::string(ex.what()) + ")";
      error(msg);
    }

    return false;
  }


  bool null() override { return true; }
  bool boolean(bool val) override {
    (void)val;
    return true;
  }
  bool binary(json::binary_t& val) override {
    (void)val;
    return true;
  }
  bool string(string_t& val) override {
    (void)val;
    return true;
  }
  bool number_integer(number_integer_t val) override {
    (void)val;
    return true;
  }

 protected:
  ArrayType odas_array;  // This will be the specific array type
  SrcType src_data;      // This will be the specific source type

  bool inside_src = false;  // Track if we're inside "src" array
  std::string current_key;

  OdasArrayCallback callback;
  ErrorCallback error;
};

class SslEventConsumer
    : public EventConsumer<odas_ros_msgs::msg::OdasSslArrayStamped, odas_ros_msgs::msg::OdasSsl> {
 public:
  SslEventConsumer(OdasArrayCallback cb, ErrorCallback on_error) : EventConsumer(cb, on_error) {}

  bool number_float(number_float_t val, const string_t& s) override {
    (void)s;

    if (inside_src) {
      if (current_key == "x") {
        src_data.x = static_cast<float>(val);
      } else if (current_key == "y") {
        src_data.y = static_cast<float>(val);
      } else if (current_key == "z") {
        src_data.z = static_cast<float>(val);
      } else if (current_key == "E") {
        src_data.e = static_cast<float>(val);
      }
    }
    return true;
  }
};

class SstEventConsumer
    : public EventConsumer<odas_ros_msgs::msg::OdasSstArrayStamped, odas_ros_msgs::msg::OdasSst> {
 public:
  SstEventConsumer(OdasArrayCallback cb, ErrorCallback on_error) : EventConsumer(cb, on_error) {}
  //bool number_integer(number_integer_t val) override {    
  bool number_unsigned(number_unsigned_t val) override {
    if (inside_src && current_key == "id") {
      src_data.id = static_cast<int>(val);
    }
    return EventConsumer::number_unsigned(val);
  }

  bool number_float(number_float_t val, const string_t& s) override {
    (void)s;
    if (inside_src) {
      if (current_key == "x") {
        src_data.x = static_cast<float>(val);
      } else if (current_key == "y") {
        src_data.y = static_cast<float>(val);
      } else if (current_key == "z") {
        src_data.z = static_cast<float>(val);
      } else if (current_key == "activity") {
        src_data.activity = static_cast<float>(val);
      }
    }
    return true;
  }
};
