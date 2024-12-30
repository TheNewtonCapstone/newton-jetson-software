//#include <catch2/catch_test_macros.hpp>
//#include "logger.h"
//#include <fstream>
//#include <string>
//
//
//TEST_CASE("Logger basic functionality", "[utils]") {
//  auto& logger = Logger::get_instance();
//  logger.set_logfile("test.log");
//
//  SECTION("info logging") {
//    logger.enable(Logger::Level::INFO);
//    newton::Logger::LOG_INFO("Test", "Info message");
//
//
//    std::ifstream file("test.log");
//    std::string line;
//    std::getline(file, line);
//    REQUIRE(line.find("INFO") != std::string::npos);
//    REQUIRE(line.find("Info message") != std::string::npos);
//  }
//
//  SECTION("debug messages are filtered when info level") {
//    logger.enable(Logger::Level::INFO);
//    Logger::LOG_DEBUG("Test", "Debug message");
//
//    std::ifstream file("test.log");
//    std::string line;
//    std::getline(file, line);
//    REQUIRE(line.find("Debug message") == std::string::npos);
//  }
//}