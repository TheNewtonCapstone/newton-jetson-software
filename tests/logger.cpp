#include "utils/logger.hpp"
#include <cassert>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

bool file_contains(const std::string& filename, const std::string& text) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    
    return content.find(text) != std::string::npos;
}

void test_log_levels() {
    auto& logger = Logger::get_instance();
    logger.set_logfile("test.log");
    
    // Clear file contents
    std::ofstream clear_file("test.log", std::ios::trunc);
    clear_file.close();
    
    // Test INFO level
    logger.enable(Logger::Level::INFO);
    Logger::INFO("Test", "Info message");
    bool found = file_contains("test.log", "Info message");
    if (!found) {
        std::cerr << "Failed to find INFO message in log file" << std::endl;
        assert(false);
    }
    
    // Test WARN level
    logger.enable(Logger::Level::WARN);
    Logger::WARN("Test", "Warning message");
    assert(file_contains("test.log", "Warning message"));
    
    // Test ERROR level
    logger.enable(Logger::Level::ERROR);
    Logger::ERROR("Test", "Error message");
    assert(file_contains("test.log", "Error message"));
}

int main() {
    std::cout << "Running logger tests...\n";
    test_log_levels();
    std::cout << "All tests passed!\n";
    return 0;
}