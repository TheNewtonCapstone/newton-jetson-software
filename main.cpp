#include "logger.h"
#include "motor_msg.h"
#include "uart.h"
#include <memory>
#include <result.h>

int main() {
    Logger::LOG_INFO("main", "Starting the application");
    com::msg::motor_msg msg(1.0, 2.0, 1);
    msg.hex_dump();
    com::serial serial("/dev/ttyUSB1", com::baudrate::B_9600);
    auto data = serial.receive();
    std::stringstream ss;
    for (auto& byte : data.get_value()) {
         ss << std::hex << static_cast<int>(byte) << " ";
    }
    Logger::LOG_INFO("main", "Received data: %s", ss.str().c_str());
    return 0;
}   