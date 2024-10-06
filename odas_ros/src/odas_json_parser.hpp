#pragma once

#include <nlohmann/json.hpp>

#include "odas_ros_msgs/msg/odas_ssl.hpp"
#include "odas_ros_msgs/msg/odas_ssl_array_stamped.hpp"
#include "odas_ros_msgs/msg/odas_sst.hpp"
#include "odas_ros_msgs/msg/odas_sst_array_stamped.hpp"

using json = nlohmann::json;

class SslEventConsumer : public json::json_sax_t {
 public:
  using OdasSslArrayCallback = std::function<void(const odas_ros_msgs::msg::OdasSslArrayStamped&)>;
  using ErrorCallback = std::function<void(const std::string& msg)>;

  SslEventConsumer(OdasSslArrayCallback cb, ErrorCallback on_error)
      : callback(cb), error(on_error) {}

  bool number_integer(number_integer_t val) override {
    if (current_key == "timeStamp") {
      odas_ssl_array.odas_time_stamp = static_cast<int>(val);
    }
    return true;
  }

  bool number_float(number_float_t val, const string_t& s) override {
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

  bool start_object(std::size_t elements) override {
    if (!inside_src) {
      odas_ssl_array = {};
    }
    return true;
  }

  bool end_object() override {
    // Add src_data to odas_sst_array.src when we finish reading a src object
    if (inside_src) {
      odas_ssl_array.sources.push_back(src_data);
    } else {
      callback(odas_ssl_array);
    }
    return true;
  }

  bool start_array(std::size_t elements) override {
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

  bool parse_error(std::size_t position, const std::string& last_token,
                   const json::exception& ex) override {
    std::string msg = "parse_error(position=" + std::to_string(position) +
                      ", last_token=" + last_token + ", ex=" + std::string(ex.what()) + ")";
    error(msg);
    return false;
  }

  bool null() override { return true; }
  bool boolean(bool val) override { return true; }
  bool binary(json::binary_t& val) override { return true; }
  bool string(string_t& val) override { return true; }
  bool number_unsigned(number_unsigned_t val) override { return true; }

 private:
  odas_ros_msgs::msg::OdasSslArrayStamped odas_ssl_array;
  odas_ros_msgs::msg::OdasSsl src_data;

  bool inside_src = false;  // Track if we're inside "src" array
  std::string current_key;

  OdasSslArrayCallback callback;
  ErrorCallback error;
};

class SstEventConsumer : public json::json_sax_t {
 public:
  using OdasSstArrayCallback = std::function<void(const odas_ros_msgs::msg::OdasSstArrayStamped&)>;
  using ErrorCallback = std::function<void(const std::string& msg)>;

  SstEventConsumer(OdasSstArrayCallback cb, ErrorCallback on_error)
      : callback(cb), error(on_error) {}

  bool number_integer(number_integer_t val) override {
    if (current_key == "timeStamp") {
      odas_sst_array.odas_time_stamp = static_cast<int>(val);
    } else if (inside_src && current_key == "id") {
      src_data.id = static_cast<int>(val);
    }
    return true;
  }

  bool number_float(number_float_t val, const string_t& s) override {
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

  bool string(string_t& val) override {
    if (inside_src && current_key == "tag") {
      // src_data.tag = val;
    }
    return true;
  }

  bool start_object(std::size_t elements) override {
    if (!inside_src) {
      odas_sst_array = {};
    }
    return true;
  }

  bool end_object() override {
    // Add src_data to odas_sst_array.src when we finish reading a src object
    if (inside_src) {
      odas_sst_array.sources.push_back(src_data);
    } else {
      callback(odas_sst_array);
    }
    return true;
  }

  bool start_array(std::size_t elements) override {
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

  bool parse_error(std::size_t position, const std::string& last_token,
                   const json::exception& ex) override {
    std::string msg = "parse_error(position=" + std::to_string(position) +
                      ", last_token=" + last_token + ", ex=" + std::string(ex.what()) + ")";
    error(msg);
    return false;
  }

  bool null() override { return true; }
  bool boolean(bool val) override { return true; }
  bool binary(json::binary_t& val) override { return true; }
  bool number_unsigned(number_unsigned_t val) override { return true; }

 private:
  odas_ros_msgs::msg::OdasSstArrayStamped odas_sst_array;
  odas_ros_msgs::msg::OdasSst src_data;

  bool inside_src = false;  // Track if we're inside "src" array
  std::string current_key;

  OdasSstArrayCallback callback;
  ErrorCallback error;
};
