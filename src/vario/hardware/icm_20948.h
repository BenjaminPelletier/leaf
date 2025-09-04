#pragma once

#include <ICM_20948.h>

#include "dispatch/message_source.h"
#include "dispatch/pollable.h"
#include "utils/state_assert_mixin.h"

class ICM20948 : public IPollable, IMessageSource, private StateAssertMixin<ICM20948> {
 public:
  // IPollable
  void update();

  // IMessageSource
  void publishTo(etl::imessage_bus* bus) { bus_ = bus; }
  void stopPublishing() { bus_ = nullptr; }

 private:
  enum class State : uint8_t {
    Uninitialized,
    WaitingForInit,
    Measuring,
  };

  void beginInit();
  void waitForInit();
  void readData();

  State state() const { return state_; }
  void onUnexpectedState(const char* action, State actual) const;
  friend struct StateAssertMixin<ICM20948>;

  State state_ = State::Uninitialized;
  uint8_t initAttempts_;
  unsigned long tLastAction_;
  ICM_20948_I2C IMU_;
  etl::imessage_bus* bus_;
};

extern ICM20948 icm20948;
