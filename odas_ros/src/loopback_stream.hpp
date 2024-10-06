#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <functional>
#include <string>

class LoopbackStream {
 public:
  using HandleInputCallback = std::function<void(std::string& data)>;
  using ErrorCallback = std::function<void(const std::string& msg)>;

  LoopbackStream(HandleInputCallback cb, int PORT, ErrorCallback on_error)
      : handle_input(cb), error(on_error), server_fd_(-1), client_fd_(-1) {
    // Set up the socket
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      error("Socket creation error");
      return;
    }

    // Assign the socket address
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(PORT);

    // Bind the socket
    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
      error("Bind failed");
      return;
    }

    if (listen(server_fd_, 3) < 0) {
      error("Listen failed");
      return;
    }
  }
  ~LoopbackStream() {
    if (server_fd_ != -1) {
      close(server_fd_);
    }
    if (client_fd_ != -1) {
      close(client_fd_);
    }
  }

  bool accept_socket() {
    // Accept the connection
    if ((client_fd_ = accept(server_fd_, (struct sockaddr*)&address_, (socklen_t*)&addrlen_)) < 0) {
      error("Accept connection");
      return false;
    }
    return true;
  }

  void read_socket() {
    const int buffer_size = 512;
    char buffer[buffer_size] = {0};

    ssize_t bytes_read = read(client_fd_, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';  // Null-terminate the string
      std::string data(buffer);

      handle_input(data);

    } else if (bytes_read == -1) {
      error("Error reading from socket");
    }
  }

 private:
  int server_fd_, client_fd_;
  struct sockaddr_in address_;
  int addrlen_ = sizeof(address_);

  HandleInputCallback handle_input;
  ErrorCallback error;
};
