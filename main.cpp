#include <csignal>
#include "udp_server.h"
#include "timer.h"

// Global flag for graceful shutdown
volatile sig_atomic_t running = 1;

// Signal handler for graceful shutdown
void signal_handler(int) {
    running = 0;
}

int main() {
    static constexpr const char* TAG = "Main";
    static constexpr uint32_t LOOP_TARGET_US = 10000; // 10ms target loop time

    // Initialize signal handling for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize logger with file output
    Logger::get_instance().enable_all();
    Logger::get_instance().set_logfile("udp_server.log");

    // Create loop timing control
    timer loop_timer("MainLoop");
    timer cycle_timer("CycleTime");

    // Network configuration
    const uint16_t JETSON_PORT = 3333;
    const uint16_t ESP_PORT = 3334;
    const std::string ESP_IP = "192.168.12.12";  // Update with your ESP32's IP

    // Initialize UDP server
    com::UDPServer server;
    auto init_result = server.init(JETSON_PORT);

    if (init_result.has_error()) {
        Logger::LOG_ERROR(TAG, "Server initialization failed: %s",
            init_result.get_error_msg().c_str());
        return 1;
    }

    Logger::LOG_INFO(TAG, "Server initialized successfully on port %d", JETSON_PORT);

    // Main communication loop
    while (running) {
        // Start timing the loop cycle
        auto cycle_start = cycle_timer.start();
        if (cycle_start.has_error()) {
            Logger::LOG_ERROR(TAG, "Failed to start cycle timer");
            break;
        }

        // Get current timestamp for message
        auto time_result = timer::apptime_us();
        if (time_result.has_error()) {
            Logger::LOG_ERROR(TAG, "Failed to get timestamp");
            continue;
        }

        // Create and send sync message
        com::Message sync_msg(time_result.get_value());
        sync_msg.set_message_type(com::MessageType::SYNC);

        // Example data to send
        uint8_t sync_data[] = { 0x01, 0x00, 0x00, 0x00 };
        auto payload_result = sync_msg.set_payload(sync_data, sizeof(sync_data));

        if (payload_result.has_error()) {
            Logger::LOG_ERROR(TAG, "Failed to set message payload");
            continue;
        }

        // Send message to ESP32
        auto send_result = server.send_message(sync_msg, ESP_IP, ESP_PORT);
        if (send_result.has_error()) {
            Logger::LOG_ERROR(TAG, "Failed to send message: %s",
                send_result.get_error_msg().c_str());
        }

        // Try to receive message from ESP32
        auto receive_result = server.receive_message();
        if (!receive_result.has_error()) {
            const auto& msg = receive_result.get_value();

            // Process received message based on type
            switch (msg.get_message_type()) {
            case com::MessageType::MOTOR_STATE: {
                // Handle motor state message
                const auto& payload = msg.get_payload();
                Logger::LOG_INFO(TAG, "Received motor state update");
                break;
            }
            case com::MessageType::RESYNC: {
                // Handle resync request
                Logger::LOG_INFO(TAG, "Received resync request");
                // Could implement resync logic here if needed
                break;
            }
            case com::MessageType::MOTOR_COMMAND: {
                // Handle motor command message
                Logger::LOG_INFO(TAG, "Received motor command");
                break;
            }
            default: {
                Logger::LOG_WARN(TAG, "Unknown message type received: %d",
                    static_cast<int>(msg.get_message_type()));
                break;
            }
            }
        }

        // Stop cycle timer and calculate loop time
        auto cycle_stop = cycle_timer.stop();
        if (!cycle_stop.has_error()) {
            auto elapsed_result = cycle_timer.elapsed_us();
            if (!elapsed_result.has_error()) {
                uint32_t elapsed_us = static_cast<uint32_t>(elapsed_result.get_value());

                // If we completed faster than our target loop time, sleep for the remainder
                if (elapsed_us < LOOP_TARGET_US) {
                    uint32_t sleep_time = LOOP_TARGET_US - elapsed_us;
                    timer::sleep_us(sleep_time);
                }
                else if (elapsed_us > LOOP_TARGET_US * 1.5) {  // Log warning if significantly over target
                    Logger::LOG_WARN(TAG, "Loop time exceeded target: %u us", elapsed_us);
                }
            }
        }
    }

    // Cleanup and final status report
    Logger::LOG_INFO(TAG, "Server shutting down...");
    auto final_time = loop_timer.stop();
    if (!final_time.has_error()) {
        auto runtime = loop_timer.elapsed_ms();
        if (!runtime.has_error()) {
            Logger::LOG_INFO(TAG, "Total runtime: %.2f ms", runtime.get_value());
        }
    }

    return 0;
}