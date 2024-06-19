/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator HAL
 *
 * MATRIX Creator HAL is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CPP_DRIVER_MICROPHONE_ARRAY_H_
#define CPP_DRIVER_MICROPHONE_ARRAY_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <valarray>

#include "./circular_queue.h"
#include "./matrix_driver.h"
#include "./pressure_data.h"

namespace matrix_hal {

static const uint32_t MIC_sampling_frequencies[][3] = {
    {8000, 374, 0},  {12000, 249, 2}, {16000, 186, 4}, {22050, 135, 5},
    {24000, 124, 5}, {32000, 92, 6},  {44100, 67, 7},  {48000, 61, 8},
    {96000, 30, 10}, {0, 0, 0}};

class MicrophoneArray : public MatrixDriver {
public:
  MicrophoneArray(bool enable_beamforming = false,
                  size_t samples_per_buffer = 512,
                  bool wait_for_buffer_full_before_reading = true);

  ~MicrophoneArray();

  void Setup(MatrixIOBus *bus);
  size_t Read();
  uint32_t SamplingRate() { return sampling_frequency_; }
  uint16_t Gain() { return gain_; }
  bool SetSamplingRate(uint32_t sampling_frequency);
  bool GetSamplingRate();
  bool GetGain();
  bool SetGain(uint16_t gain);
  void ReadConfValues();
  void ShowConfiguration();
  uint16_t Channels() { return kMicrophoneChannels; }
  uint32_t NumberOfSamples() {
    // This function returns the number of samples read per channel.

    // If we have not read any samples, return the maximum possible buffer size
    // (as it's used to preallocate a buffer in the examples)
    if (use_read_cv_ || !first_read_) {
      return kMicarrayBufferSize / kMicrophoneChannels;
    } else {
      return std::lround(std::floor(num_samples_read_ / kMicrophoneChannels));
    }
  }

  int16_t &Raw(int16_t sample, int16_t channel) {
    if (use_read_cv_) {
      return raw_data_[channel * number_of_samples_internal() + sample];
    } else if (!use_read_cv_ && num_samples_read_ == kMicarrayBufferSize) {
      return read_samples_[channel * number_of_samples_internal() + sample];
    } else {
      return read_samples_[sample * kMicrophoneChannels + channel];
    }
  }

  int16_t &At(int16_t sample, int16_t channel) {
    if (!enable_beamforming_)
      return Raw(sample, channel);
    return delayed_data_[sample * kMicrophoneChannels + channel];
  }

  // call at own peril if beamforming is disabled
  int16_t &Beam(int16_t sample) { return beamformed_[sample]; }

  void CalculateDelays(float azimutal_angle, float polar_angle,
                       float radial_distance_mm = 100.0,
                       float sound_speed_mmseg = 320 * 1000.0);

private:
  std::unique_lock<std::mutex> lock_;
  //  delay and sum beamforming result
  std::valarray<int16_t> beamformed_;
  std::valarray<int16_t> raw_data_;
  std::valarray<int16_t> delayed_data_;
  std::valarray<int16_t> fir_coeff_;
  int16_t gain_;
  uint32_t sampling_frequency_;
  bool enable_beamforming_;
  bool use_read_cv_;
  size_t last_read_sample_{0};
  bool first_read_{false};
  std::valarray<int16_t> read_samples_;
  // Total number of samples read (all channels, not per channel)
  uint32_t num_samples_read_{0};

  uint16_t kMicarrayBufferSize{512 * 8};
  const uint16_t kMicrophoneArrayIRQ{22}; // GPIO06 - WiringPi:22
  const uint16_t kMicrophoneChannels{8};

  // beamforming delay and sum support
  std::valarray<CircularQueue<int16_t>> fifos_;

  int16_t &raw_data_internal(int16_t sample, int16_t channel) {
    return raw_data_[channel * number_of_samples_internal() + sample];
  }
  uint32_t number_of_samples_internal() {
    return kMicarrayBufferSize / kMicrophoneChannels;
  }
};
}; // namespace matrix_hal
#endif // CPP_DRIVER_MICROPHONE_ARRAY_H_
