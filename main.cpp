#include "logger.h"
#include "motor_msg.h"
#include "uart.h"
#include <memory>
#include <result.h>

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // Configure server address
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(3333);

    // Bind socket
    if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "UDP Server listening on port 3333..." << std::endl;

    // Main receive loop
    char buffer[1024];
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    while (true) {
        memset(buffer, 0, sizeof(buffer));

        // Receive data
        ssize_t recvLen = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
            (struct sockaddr*)&clientAddr, &clientAddrLen);

        if (recvLen > 0) {
            // Get client IP
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIP, INET_ADDRSTRLEN);

            // Print received message
            std::cout << "Message from " << clientIP << ": " << buffer << std::endl;
        }
    }

    close(sock);
    return 0;
}