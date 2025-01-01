#pragma once
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "result.h"
#include "logger.h"
#include "timer.h"
#include "base_msg.h"

namespace com {

  class UDPServer {
  public:
      // Constants for configuration
    static constexpr const char* TAG = "UDPServer";
    static constexpr int RECEIVE_TIMEOUT_MS = 1;  // Short timeout for real-time operation

    UDPServer() : socket_fd_(-1), comm_timer_("UDP_Communication") {}

    ~UDPServer() {
      if (socket_fd_ >= 0) {
        close(socket_fd_);
      }
    }

    result<void> init(uint16_t port) {
        // Create UDP socket
      socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
      if (socket_fd_ < 0) {
        Logger::LOG_ERROR(TAG, "Socket creation failed");
        return error("Socket creation failed");
      }

      // Set socket options for timeout
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = RECEIVE_TIMEOUT_MS * 1000;
      if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        Logger::LOG_ERROR(TAG, "Failed to set socket timeout");
        return error("Socket timeout configuration failed");
      }

      // Configure address
      struct sockaddr_in server_addr;
      memset(&server_addr, 0, sizeof(server_addr));
      server_addr.sin_family = AF_INET;
      server_addr.sin_addr.s_addr = INADDR_ANY;
      server_addr.sin_port = htons(port);

      // Bind socket
      if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        Logger::LOG_ERROR(TAG, "Socket bind failed on port %d", port);
        return error("Socket bind failed");
      }

      // Start communication timer
      auto timer_result = comm_timer_.start();
      if (timer_result.has_error()) {
        Logger::LOG_ERROR(TAG, "Failed to start communication timer");
        return error("Timer initialization failed");
      }

      Logger::LOG_INFO(TAG, "UDP Server initialized on port %d", port);
      return success();
    }

    result<void> send_message(const Message& msg, const std::string& target_ip, uint16_t target_port) {
      struct sockaddr_in target_addr;
      memset(&target_addr, 0, sizeof(target_addr));
      target_addr.sin_family = AF_INET;
      target_addr.sin_port = htons(target_port);

      if (inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr) <= 0) {
        Logger::LOG_ERROR(TAG, "Invalid target IP: %s", target_ip.c_str());
        return error("Invalid IP address");
      }

      uint8_t buffer[Message::MAX_MSG_SIZE];
      uint16_t msg_size = msg.get_full_message(buffer, Message::MAX_MSG_SIZE);

      ssize_t sent_bytes = sendto(socket_fd_, buffer, msg_size, 0,
        (struct sockaddr*)&target_addr, sizeof(target_addr));

      if (sent_bytes < 0) {
        Logger::LOG_ERROR(TAG, "Failed to send message to %s:%d", target_ip.c_str(), target_port);
        return error("Message transmission failed");
      }

      return success();
    }

    result<Message> receive_message() {
      struct sockaddr_in client_addr;
      socklen_t client_addr_len = sizeof(client_addr);
      uint8_t buffer[Message::MAX_MSG_SIZE];

      ssize_t recv_bytes = recvfrom(socket_fd_, buffer, Message::MAX_MSG_SIZE, 0,
        (struct sockaddr*)&client_addr, &client_addr_len);

      if (recv_bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          Logger::LOG_DEBUG(TAG, "No message received");
        }
        Logger::LOG_ERROR(TAG, "Message reception failed");

      }

      // Get current timestamp for message validation
      auto time_result = timer::apptime_us();
      if (time_result.has_error()) {
        return error<Message>("Failed to get timestamp");
      }

      // Create and validate message
      Message received_msg(time_result.get_value());
      if (!received_msg.parse_message(buffer, recv_bytes)) {
        Logger::LOG_ERROR(TAG, "Received invalid message");
        return error<Message>("Invalid message format");
      }

      return success<Message>(received_msg);
    }

    result<double> get_comm_time_ms() const {
      return comm_timer_.elapsed_ms();
    }

  private:
    int socket_fd_;
    timer comm_timer_;
  };

} // namespace com