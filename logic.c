#include "spi.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

int main() {
  spi_t spi;
  const char* message = "hello";
  int retv;

  while (1) {
  // Initialize SPI
  // Using default mode 0 (CPOL=0, CPHA=0) as shown in your screenshot
  // 8 bits per word as shown in screenshot
  // 1MHz speed for reliable capture on logic analyzer
    retv = spi_init(&spi,
      "/dev/spidev0.0",  // Device path - adjust if needed
      0,                  // Mode 0 (CPOL=0, CPHA=0)
      8,                  // 8 bits per word
      1000000);          // 1MHz speed

    if (retv != SPI_ERR_NONE) {
      printf("Failed to initialize SPI: error %d\n", retv);
      return 1;
    }

    printf("SPI initialized successfully\n");

    // Send the message
    retv = spi_write(&spi, message, strlen(message));

    if (retv < 0) {
      printf("Failed to write message: error %d\n", retv);
    }
    else {
      printf("Successfully wrote %d bytes\n", retv);
    }
    sleep(1);

  }
  // Clean up
  spi_free(&spi);
  return 0;
}