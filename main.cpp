#include "utils/logger.h"
#include "communication/message.h"
#include "communication/uart.h"
#include "communication/handler.h"
#include <memory>


int main() {
    Logger::INFO("Main", "Starting application");
    com::message msg;
    com::uart_settings settings = {
        .address = "/dev/ttyUSB0",
        .baudrate = com::uart_baudrate::B_230400,
        .timeout_ms = 1000
    };

    auto serial = std::make_unique<com::uart>();
    com::status res = serial->connect(settings);
    if (res != com::status::OK) {
        Logger::ERROR("Main", "Failed to connect to UART");
        return -1;
    }

    // create Transport
    com::MessageHandler handler(std::move(serial));

    struct SensorData {
        float temperature;
        float humidity;
        uint32_t timestamp;
    };
    SensorData data{ 25.5f, 60.0f, 123456789 };
    handler.send(data);


    Logger::INFO("Main", "Message size: %i", sizeof(msg));

    return 0;
}