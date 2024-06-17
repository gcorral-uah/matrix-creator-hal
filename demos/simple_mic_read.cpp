#include <array>
#include <sys/stat.h>
#include <sys/types.h>
// Linux file control options
#include <fcntl.h>
// System calls
#include <unistd.h>
// Input/output streams and functions
#include <fstream>
#include <iostream>

// Communicates with MATRIX device
#include "../cpp/driver/matrixio_bus.h"
// Interfaces with microphone array
#include "../cpp/driver/microphone_array.h"
// Enables using FIR filter with microphone array
#include "../cpp/driver/microphone_core.h"

#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

#include <cmath>
#include <vector>

int main() {

  const int seconds_to_record = 3;

  // Create MatrixIOBus object for hardware communication
  matrix_hal::MatrixIOBus bus;
  // Initialize bus and exit program if error occurs
  if (!bus.Init())
    return 1;

  // Set user flags from gflags as variables
  int sampling_rate = 96000;
  int gain = -1;

  // Create MicrophoneArray object
  // matrix_hal::MicrophoneArray microphone_array{false, 512, false};
  matrix_hal::MicrophoneArray microphone_array{};
  // Set microphone_array to use MatrixIOBus bus
  microphone_array.Setup(&bus);
  // Set microphone sampling rate
  microphone_array.SetSamplingRate(sampling_rate);

  // If gain is positive, set the gain
  if (gain > 0) {
    microphone_array.SetGain(gain);
  }

  // Log gain_ and sampling_frequency_ variables
  microphone_array.ShowConfiguration();

  // Create MicrophoneCore object
  matrix_hal::MicrophoneCore microphone_core(microphone_array);
  // Set microphone_core to use MatrixIOBus bus
  microphone_core.Setup(&bus);

  // Create a buffer array for microphone input
  int16_t buffer[microphone_array.Channels()]
                [microphone_array.SamplingRate() +
                 microphone_array.NumberOfSamples()];
  // Create an array of streams to write microphone data to files
  std::ofstream os[microphone_array.Channels()];
  // For each microphone channel (+1 for beamforming), make a file and open it
  for (uint16_t c = 0; c < microphone_array.Channels(); c++) {
    // Set filename for microphone output
    std::string filename = "mic_" +
                           std::to_string(microphone_array.SamplingRate()) +
                           "_s16le_channel_" + std::to_string(c) + ".raw";
    // Create and open file
    os[c].open(filename, std::ofstream::binary);
  }

  // Counter variable for tracking recording time
  uint32_t samples = 0;
  // For recording duration
  for (int s = 0; s < seconds_to_record; s++) {
    // Endless loop
    while (true) {
      // Read microphone stream data
      auto start = std::chrono::high_resolution_clock::now();
      // Read microphone stream data
      microphone_array.Read();
      auto stop = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> diferencia_tiempo =
          stop - start;
      float Ts = static_cast<float>(diferencia_tiempo.count());
      std::cout << "Tiempo de lectura MIC: " << Ts << " miliseconds"
                << std::endl;

      auto start1 = std::chrono::high_resolution_clock::now();
      // For number of samples
      for (uint32_t s = 0; s < microphone_array.NumberOfSamples(); s++) {
        // For each microphone
        for (uint16_t c = 0; c < microphone_array.Channels(); c++) {
          // Send microphone data to buffer
          buffer[c][samples] = microphone_array.At(s, c);
        }
        // Increment samples for buffer write
        samples++;
      }
      auto stop1 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> dif_tiempo1 = stop1 - start1;
      float Ts_1 = static_cast<float>(dif_tiempo1.count());
      std::cout << "Tiempo escritura en array: " << Ts_1 << " miliseconds "
                << " for " << microphone_array.NumberOfSamples() << " samples "
                << std::endl;

      // Write a part of the file once number of samples is >= sampling rate,
      // (which indicates that a duration of a second at a minimum have
      // passed).
      if (samples >= microphone_array.SamplingRate()) {
        // For each microphone channel
        for (uint16_t c = 0; c < microphone_array.Channels(); c++) {
          // Write to recording file
          os[c].write((const char *)buffer[c], samples * sizeof(int16_t));
        }
        // Set samples to zero for loop to fill buffer
        samples = 0;
        break;
      }
    }
  }
  return 0;
}
