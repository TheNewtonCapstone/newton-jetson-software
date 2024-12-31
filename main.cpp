#include "logger.h"
#include "motor_msg.h"
#include "uart.h"
#include <memory>
#include <result.h>

int main() {
    Logger::LOG_INFO("main", "Starting the application");
    com::msg::motor_msg msg(1.0, 2.0, 1);

    return 0;
}   