#include <array>
#include <sys/stat.h>
#include <sys/types.h>
// Linux file control options
#include <fcntl.h>
// System calls
#include <unistd.h>
// Input/output streams and functions
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

  uint64_t loop_counter = 0;

  // Create MatrixIOBus object for hardware communication
  matrix_hal::MatrixIOBus bus;
  // Initialize bus and exit program if error occurs
  if (!bus.Init())
    return 1;

  // Set user flags from gflags as variables
  int sampling_rate = 96000;
  int gain = -1;

  // Create MicrophoneArray object
  matrix_hal::MicrophoneArray microphone_array{false, 512, false};
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

  // For ensuring there are no hangs
  while (true) {
    loop_counter++;

    // Read microphone stream data
    // auto start = std::chrono::high_resolution_clock::now();

    microphone_array.Read();

    // auto stop = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> diferencia_tiempo = stop -
    // start;

    // float Ts = static_cast<float>(diferencia_tiempo.count());
    // std::cout << "Tiempo de lectura MIC: " << Ts << " miliseconds" <<
    // std::endl;

    // For each microphone

    auto start1 = std::chrono::high_resolution_clock::now();

    for (size_t c = 0; c < microphone_array.Channels(); c++) {
      std::cerr << "Writing in loop " << loop_counter << " to mic " << c
                << std::endl;
      // For number of samples
      for (size_t s = 0; s < microphone_array.NumberOfSamples(); s++) {
        buffer[c][s] = microphone_array.At(s, c);
      }
    }

    auto stop1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dif_tiempo1 = stop1 - start1;
    float Ts_1 = static_cast<float>(dif_tiempo1.count());
    std::cout << "Tiempo escritura en array: " << Ts_1 << " miliseconds "
              << " for " << microphone_array.NumberOfSamples() << " samples "
              << std::endl;
    auto buf = std::vector<int16_t>{};
    buf.assign(&buffer[0][0], &buffer[microphone_array.Channels()]
                                     [(microphone_array.SamplingRate() +
                                       microphone_array.NumberOfSamples())]);
    std::cerr << buf.size() << std::endl;
  }

  return 0;
}
