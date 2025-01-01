#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <signal.h>

class UDPServer {
private:
  int sock;
  struct sockaddr_in serverAddr;
  bool running;
  const int BUFFER_SIZE = 1024;

  std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    char buf[100];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
    return std::string(buf);
  }

public:
  UDPServer(int port = 3333) : sock(-1), running(false) {
      // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    // Configure server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    // Enable address reuse
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
      throw std::runtime_error("Failed to set socket options");
    }

    // Bind socket
    if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
      throw std::runtime_error("Failed to bind socket");
    }
  }

  ~UDPServer() {
    if (sock >= 0) {
      close(sock);
    }
  }

  void start() {
    running = true;
    char buffer[BUFFER_SIZE];
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    std::cout << "UDP Server listening on port " << ntohs(serverAddr.sin_port) << std::endl;

    while (running) {
        // Clear buffer
      memset(buffer, 0, BUFFER_SIZE);

      // Receive data
      ssize_t recvLen = recvfrom(sock, buffer, BUFFER_SIZE - 1, 0,
        (struct sockaddr*)&clientAddr, &clientAddrLen);

      if (recvLen > 0) {
        buffer[recvLen] = '\0';  // Null terminate the received data

        // Get client IP address
        char clientIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIP, INET_ADDRSTRLEN);

        // Print received message with timestamp
        std::cout << "[" << getCurrentTimestamp() << "] "
          << "Message from " << clientIP << ":"
          << ntohs(clientAddr.sin_port) << ": "
          << buffer << std::endl;

 // Here you can add custom processing for the received data
 // For example: processData(buffer);
      }
    }
  }

  void stop() {
    running = false;
  }
};

// Signal handler for graceful shutdown
UDPServer* globalServer = nullptr;
void signalHandler(int signum) {
  if (globalServer) {
    std::cout << "\nShutting down server..." << std::endl;
    globalServer->stop();
  }
}
