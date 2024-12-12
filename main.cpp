#include "logger.hpp"

int main() {
    newton::Logger::INFO("Motors", "Speed: {:.2f} RPM", 123.456);
    return 0;
}