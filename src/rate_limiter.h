#ifndef HALMET_SRC_RATE_LIMITER_H_
#define HALMET_SRC_RATE_LIMITER_H_

#include "sensesp/transforms/transform.h"

namespace sensesp {

/**
 * @brief Transform that limits the output rate to a specified minimum delay.
 *
 * @tparam T
 */
template <typename T>
class RateLimiter : public Transform<T, T> {
 public:
  RateLimiter(unsigned int min_delay_ms, String config_path = "")
      : Transform<T, T>(config_path), min_delay_ms_{min_delay_ms} {}

  virtual void set_input(T input, uint8_t input_channel = 0) override {
    unsigned long current_time = millis();
    if (current_time - last_output_time_ > min_delay_ms_) {
      this->emit(input);
      last_output_time_ = current_time;
    }
  }

 private:
  unsigned int min_delay_ms_;
  unsigned long last_output_time_ = 0;
};

}  // namespace sensesp

#endif /* HALMET_SRC_RATE_LIMITER_H_ */
