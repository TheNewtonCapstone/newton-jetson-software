#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <vector>
#include <array>
#include <string>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <thread>
#include <chrono>
#include <ctime>
#include <math.h>

#define DEBUG_LOG

using namespace std::chrono_literals;

struct ActuatorState {
  unsigned long jetson_send_time; // timestamp in microseconds
  unsigned long jetson_receive_time;
  unsigned long esp_receive_time;
  unsigned long esp_measure_time;

  float commanded_torque; // normalized used by issac sim
  int pulse_count; // info from encoder
  float velocity;
 float pwm;
  bool direction;
};

constexpr int RX_BUFFER_SIZE = 255;
constexpr int TX_BUFFER_SIZE = 32;


constexpr auto PERIOD = 20ms; // 20 milliseconds for 50 Hz

// function prototypes
int read_serial(int file_desc, std::vector<float>&);
int send_serial(int file_desc, std::array<float, 2>&);

ActuatorState read_actuator_state(int file_desc);
int send_actuator_command(int file_desc, float command);


int config_serial();
int action_to_pwm(float action);
std::string get_cwd();
std::string get_log_time();

int main(int argc, char const* argv[]) {
  int file_desc;
  std::string work_dir = get_cwd();

  std::ofstream log_file;
  std::string log_file_path = work_dir + "/logs/" + get_log_time() + ".csv";
  log_file.open(log_file_path);

  log_file << "jetson_send_time,esp_receive_time,measure_time,jetson_receive_time,timestamp_us,commanded_torque,pulse_count,velocity,pwm,direction" << std::endl;
  try {
    file_desc = config_serial();
    auto current_time = std::chrono::steady_clock::now();
    std::cout << "Step response test" << std::endl;
    /**
     * Step response test
     * measure the response of the system to a step input duuuh
     * in other words we want to measure how the sys react to sudden change
     * we are interested in the following parameters
     *  - rise time : how quickly the sys reaches the commanded torque
     *  -settling time : how long until oscilliation stops (not sure if we will see this)
     *  - overshoot : how much the sys goes beyond the commanded torque
     *  - steady state error : how much the sys deviates from the commanded torqueA
     *
     *  are there any dead zones(areas where the sys does not respond to input aka below .35)
     *  are there any satutation zones?
     */
    std::array<float, 3> amplitudes = {0.3, 0.6, 0.9};
    for(float amp : amplitudes) {
      float t = 0.f;
      while(t < 5.0f) {
        // split the step into 2 directions
        float  cmd = (t < 2.5f)? amp : -amp;
        auto send_time = std::chrono::high_resolution_clock::now();
        send_actuator_command(file_desc, cmd);
        auto response = read_actuator_state(file_desc);
        auto receive_time = std::chrono::high_resolution_clock::now();

        response.jetson_send_time = std::chrono::duration_cast<std::chrono::microseconds>(send_time.time_since_epoch()).count();
        response.jetson_receive_time = std::chrono::duration_cast<std::chrono::microseconds>(receive_time.time_since_epoch()).count();
        log_file << response.jetson_send_time
        << "," << response.esp_receive_time
        << "," << response.esp_measure_time
        << "," << response.jetson_receive_time
        << "," << response.jetson_receive_time
        << "," << response.commanded_torque
        << "," << response.pulse_count
        << "," << response.velocity
        << "," << response.pwm
        << "," << response.direction << std::endl;

        t+= 0.01f;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

  /* sine wave test
   * TODO : not well understood yet
   */

    {
      std::cout << "Sine wave test, WOOOOOOOO!" << std::endl;
      float t = 0.f;
      constexpr float min_freq = 0.1f, max_freq = 10.f, duration = 60.f;
      constexpr float increment = (max_freq - min_freq) / duration;
      while(t < duration) {
        float freq = min_freq + increment * t;
        float cmd = std::sin(2 * M_PI * freq * t);
        auto send_time = std::chrono::high_resolution_clock::now();
        send_actuator_command(file_desc, cmd);
        auto response = read_actuator_state(file_desc);
        auto receive_time = std::chrono::high_resolution_clock::now();

        response.jetson_send_time = std::chrono::duration_cast<std::chrono::microseconds>(send_time.time_since_epoch()).count();
        response.jetson_receive_time = std::chrono::duration_cast<std::chrono::microseconds>(receive_time.time_since_epoch()).count();
        log_file << response.jetson_send_time
        << "," << response.esp_receive_time
        << "," << response.esp_measure_time
        << "," << response.jetson_receive_time
        << "," << response.jetson_receive_time
        << "," << response.commanded_torque
        << "," << response.pulse_count
        << "," << response.velocity
        << "," << response.pwm
        << "," << response.direction << std::endl;

        t+= 0.01f;
        freq += increment;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    /*
     * Random Commands
     */
    {

    }

  }
  catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    close(file_desc);
  }

  return 0; // success
};


ActuatorState read_actuator_state(int file_desc) {
  std::array<char, RX_BUFFER_SIZE> buffer{};
  ActuatorState state{};
  const ssize_t num_bytes = read(file_desc, buffer.data(), RX_BUFFER_SIZE);

  if (num_bytes < 0) {
    throw std::runtime_error("Error reading from serial port");
  }
  std::cout << "Received " << num_bytes << " bytes\n";
  //todo : correct the position of end character
  if (buffer[0] != 's' || buffer[15] != 'e') {
    throw std::runtime_error("Corrupted data");
  }

  memcpy(&state, buffer.data() + 3, sizeof(ActuatorState));
  return state;
}
int send_actuator_command(int file_desc, float command) {
  std::array<char, TX_BUFFER_SIZE> buffer{};

  buffer[0] = 's'; // starting character
  buffer[1] = 'f'; // the type of data being sent
  buffer[2] = 1;// the number of data being sent
  buffer[31] = 'e';
  std::memcpy(buffer.data() + 3, &command, sizeof(float));
  return write(file_desc, buffer.data(), RX_BUFFER_SIZE);
}

int config_serial() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int file_desc = open("/dev/ttyTHS1", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(file_desc, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B230400);
  cfsetospeed(&tty, B230400);

  // Save tty settings, also checking for error
  if (tcsetattr(file_desc, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return file_desc;
}

int action_to_pwm(float action) {
  constexpr float MIN_DUTY = 0.375f;
  constexpr float DUTY_RANGE = 1.f - MIN_DUTY;
  return static_cast<int>((std::abs(action) * DUTY_RANGE + MIN_DUTY) * 255.f);
}

std::string get_cwd() {
  std::filesystem::path cwd = std::filesystem::current_path();
  std::cout << "Current Working directory is : " << cwd << std::endl;
  std::filesystem::path parent = cwd.parent_path();
  std::cout << "Parent directory is : " << parent << std::endl;
  return parent.string();
}

std::string get_log_time() {
  std::time_t t = std::time(0);
  std::tm* now = std::localtime(&t);
  std::ostringstream oss;
  oss << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' << now->tm_mday << "_" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec;
  return oss.str();
}