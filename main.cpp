#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>

int main() {
    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // Configure Jetson server address (for receiving)
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(3333);

    // Configure ESP32 address (for sending)
    struct sockaddr_in espAddr;
    memset(&espAddr, 0, sizeof(espAddr));
    espAddr.sin_family = AF_INET;
    espAddr.sin_addr.s_addr = inet_addr("192.168.12.12"); // Replace X with ESP32's IP
    espAddr.sin_port = htons(3334);

    // Bind socket
    if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "UDP Server running. Sending and receiving on different ports..." << std::endl;

    char buffer[1024];
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    unsigned long counter = 0;

    while (true) {
        // Send counter to ESP32
        std::string message = "Jetson Counter: " + std::to_string(counter);
        sendto(sock, message.c_str(), message.length(), 0,
            (struct sockaddr*)&espAddr, sizeof(espAddr));

     // Check for incoming data
        memset(buffer, 0, sizeof(buffer));
        ssize_t recvLen = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
            (struct sockaddr*)&clientAddr, &clientAddrLen);

        if (recvLen > 0) {
            // Get client IP
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIP, INET_ADDRSTRLEN);

            // Print received message
            std::cout << "Message from " << clientIP << ": " << buffer << std::endl;
        }

        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(sock);
    return 0;
}