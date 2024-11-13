#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0.0"  // Adjust if needed
#define BUFFER_SIZE 32

int main(void) {
  int fd;
  int ret = 0;

  // Open SPI device
  fd = open(SPI_DEVICE, O_RDWR);
  if (fd < 0) {
    perror("Can't open SPI device");
    return 1;
  }

  // Configure SPI mode
  uint8_t mode = SPI_MODE_0;
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret < 0) {
    perror("Can't set SPI mode");
    return 1;
  }

  // Configure bits per word
  uint8_t bits = 8;
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret < 0) {
    perror("Can't set bits per word");
    return 1;
  }

  // Configure speed
  uint32_t speed = 500000;
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret < 0) {
    perror("Can't set speed");
    return 1;
  }

  // Prepare buffers
  uint8_t tx_buffer[BUFFER_SIZE] = "Hello from Jetson!";
  uint8_t rx_buffer[BUFFER_SIZE] = { 0 };

  // Prepare transfer
  struct spi_ioc_transfer transfer = {
      .tx_buf = (unsigned long)tx_buffer,
      .rx_buf = (unsigned long)rx_buffer,
      .len = BUFFER_SIZE,
      .speed_hz = speed,
      .bits_per_word = bits,
      .delay_usecs = 0,
  };

  printf("Starting SPI communication...\n");

  // Main loop
  while (1) {
      // Perform transfer
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    if (ret < 0) {
      perror("SPI transfer failed");
    }
    else {
      printf("Sent: %s\n", tx_buffer);
      printf("Received: %s\n", rx_buffer);
    }

    sleep(1);  // Wait 1 second before next transfer
  }

  close(fd);
  return 0;
}