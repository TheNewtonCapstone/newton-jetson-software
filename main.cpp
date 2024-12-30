#include "logger.h"
#include "message.h"
#include "uart.h"
#include "types.h"
#include <memory>
#include <result.h>

int main() {
    Logger::LOG_INFO("main", "Starting the application");
    // create transport layer
    auto uart = std::make_shared<com::serial>("/dev/ttyUSB0", com::baudrate::B_115200);
    if(uart->is_connected()) {
        Logger::LOG_INFO("main", "Connected to the device");
    } else {
        Logger::LOG_ERROR("main", "Failed to connect to the device");
    }
    // create a message handle
   auto handler = com::handler<MotorCommand>(uart);

    
    MotorCommand cmd{
    .velocity = 10.0f,
    .position = 90.0f,
    .motor_id = 1
};
    com::message<MotorCommand> msg(cmd, com::message_type::MOTOR_COMMAND);


    return 0;
}   