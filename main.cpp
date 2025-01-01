#include "logger.h"
int main() {
    newton::Logger::LOG_INFO("Motors", "Speed: {:.2f} RPM", 123.456);
    return 0;
}