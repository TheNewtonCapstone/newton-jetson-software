#pragma once
#include "communication/message.h"
#include "utils/logger.hpp"
#include <sstream>
#include <iomanip>

namespace diagnostics {
  class MessageDiagnostics {
  public:
    static void hex_dump(const Message& msg, const std::string& context = "") {
      if (msg.body.data.empty()) {
        Logger::ERROR("MessageDiagnostics", "Cannot dump empty message");
        return;
      }

      //header
      std::stringstream header;
      header << "\nMessage Diagnostic Information";
      if (!context.empty()) {
        header << " (" << context << ")";
      }
      Logger::INFO("MessageDiagnostics", header.str().c_str());

      // Display message metadata
      std::stringstream meta;
      meta << "Size: " << msg.body.data.size() << " bytes";
      Logger::INFO("MessageDiagnostics", meta.str().c_str());

      // Format the hex dump with both hex and ASCII representation
      std::stringstream hex_stream;
      hex_stream << "Hex dump:\n";

      for (size_t i = 0; i < msg.body.data.size(); i++) {
        if (i % 16 == 0) {
          if (i > 0) {
            print_ascii_representation(hex_stream, msg.body.data, i - 16, i);
          }
          hex_stream << std::setw(4) << std::setfill('0') << std::hex << i << ": ";
        }

        // Print hex value
        hex_stream << std::setw(2) << std::setfill('0') << std::hex
          << static_cast<int>(msg.body.data[i]) << " ";

      // End of data - print remaining ASCII representation
        if (i == msg.body.data.size() - 1) {
            // Pad with spaces if we're not at the end of a 16-byte line
          for (size_t j = 0; j < 15 - (i % 16); j++) {
            hex_stream << "   ";
          }
          print_ascii_representation(hex_stream, msg.body.data,
            i - (i % 16), msg.body.data.size());
        }
      }

      Logger::INFO("MessageDiagnostics", hex_stream.str().c_str());
    }


    static void validate_structure(const Message& msg) {
      std::stringstream ss;
      ss << "\nStructure Validation:";

      bool header_valid = true;  // Add your header validation logic
      ss << "\n- Header: " << (header_valid ? "Valid" : "Invalid");

      bool data_valid = !msg.body.data.empty();
      ss << "\n- Body: " << (data_valid ? "Valid" : "Empty/Invalid");


      Logger::INFO("MessageDiagnostics", ss.str().c_str());
    }

  private:
    static void print_ascii_representation(std::stringstream& ss,
      const std::array<unsigned char, Message>data,
      size_t start, size_t end) {
      ss << "  |";
      for (size_t i = start; i < end; i++) {
        char c = static_cast<char>(data[i]);
        // Print only printable ASCII characters, replace others with a dot
        ss << (c >= 32 && c <= 126 ? c : '.');
      }
      ss << "|\n";
    }

  };

}