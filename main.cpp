#include "utils/logger.hpp"

int main(){
    Logger::INFO("Motors", "Speed: {:.2f} RPM", 123.456);
    return 0;
}