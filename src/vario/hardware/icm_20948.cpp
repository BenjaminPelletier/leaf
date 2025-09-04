/*
 * TDK InvenSense ICM-20498
 * 6 DOF Gyro+Accel plus 3-axis mag
 *
 */

#include "hardware/icm_20948.h"

#include <ICM_20948.h>

#include "diagnostics/fatal_error.h"
#include "dispatch/message_types.h"
#include "utils/magic_enum.h"

#define DEBUG_IMU 0

#define WIRE_PORT Wire
#define SERIAL_PORT Serial
#define AD0_VAL 0  // I2C address bit

ICM20948 icm20948;

void ICM20948::beginInit() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  if (DEBUG_IMU) IMU_.enableDebugging();  // enable helpful debug messages on Serial
  initAttempts_ = 0;
  tLastAction_ = millis() - 500;
  state_ = State::WaitingForInit;
}

void ICM20948::waitForInit() {
  if (millis() - tLastAction_ < 500) {
    return;
  }

  IMU_.begin(WIRE_PORT, AD0_VAL);
  if (DEBUG_IMU) {
    SERIAL_PORT.print(F("Initialization of the IMU returned: "));
    SERIAL_PORT.println(IMU_.statusString());
  }
  ICM_20948_Status_e status = IMU_.status;
  if (status != ICM_20948_Stat_Ok) {
    initAttempts_++;
    if (initAttempts_ >= 3) {
      fatalError("IMU could not be initialized (status %u)", status);
    }
    if (DEBUG_IMU) {
      SERIAL_PORT.println("Trying again...");
    }
    tLastAction_ = millis();
    return;
  }

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g.
  // to change the sample rate
  status = IMU_.initializeDMP();
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU initializeDMP failed (%u)", status);
  }

  // Enable the DMP orientation and accelerometer sensors
  status = IMU_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU enableDMPSensor orientation failed (%u)", status);
  }
  status = IMU_.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER);
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU enableDMPSensor acceleration failed (%u)", status);
  }

  // Configuring DMP to output data at multiple ODRs:
  status = IMU_.setDMPODRrate(DMP_ODR_Reg_Quat9, 2);  // Set to the maximum
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU setDMPODRrate Quat9 failed (%u)", status);
  }
  status = IMU_.setDMPODRrate(DMP_ODR_Reg_Accel, 2);  // Set to the maximum
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU setDMPODRrate Accel failed (%u)", status);
  }

  // Enable the FIFO
  status = IMU_.enableFIFO();
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU enableFIFO failed (%u)", status);
  }

  // Enable the DMP
  status = IMU_.enableDMP();
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU enableDMP failed (%u)", status);
  }

  // Reset DMP
  status = IMU_.resetDMP();
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU resetDMP failed (%u)", status);
  }

  // Reset FIFO
  status = IMU_.resetFIFO();
  if (status != ICM_20948_Stat_Ok) {
    fatalError("IMU resetFIFO failed (%u)", status);
  }

  state_ = State::Measuring;
}

void ICM20948::update() {
  if (state_ == State::Uninitialized) {
    beginInit();
  } else if (state_ == State::WaitingForInit) {
    waitForInit();
  } else if (state_ == State::Measuring) {
    readData();
  } else {
    fatalError("ICM20948::update with unsupported state %s (%u)", nameOf(state_).c_str(), state_);
  }
}

void ICM20948::readData() {
  icm_20948_DMP_data_t data;
  IMU_.readDMPdataFromFIFO(&data);  // TODO: consider rate-limiting this operation

  if ((IMU_.status != ICM_20948_Stat_Ok) &&
      (IMU_.status != ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {
    return;
  }

  MotionUpdate update(millis());
  if ((data.header & DMP_header_bitmap_Quat9) > 0) {
    // Scale to +/- 1
    update.qx = ((double)data.Quat9.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
    update.qy = ((double)data.Quat9.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
    update.qz = ((double)data.Quat9.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30
    update.hasOrientation = true;
  }
  if ((data.header & DMP_header_bitmap_Accel) > 0) {
    // Scale to Gs
    update.ax = ((double)data.Raw_Accel.Data.X) / 8192.0;
    update.ay = ((double)data.Raw_Accel.Data.Y) / 8192.0;
    update.az = ((double)data.Raw_Accel.Data.Z) / 8192.0;
    update.hasAcceleration = true;
  }
  etl::imessage_bus* bus = bus_;
  if ((update.hasOrientation || update.hasAcceleration) && bus) {
    // Invalidate insane orientation data
    if (update.hasOrientation) {
      if (isnan(update.qx) || isinf(update.qx) || update.qx < -1.1 || update.qx > 1.1 ||
          isnan(update.qy) || isinf(update.qy) || update.qy < -1.1 || update.qy > 1.1 ||
          isnan(update.qz) || isinf(update.qz) || update.qz < -1.1 || update.qz > 1.1) {
        char msg[100];
        snprintf(msg, sizeof(msg),
                 "ICM20948 invalid orientation Q1=%X; Q2=%X; Q3=%X (%.2f, %.2f, %.2f)",
                 data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, update.qx, update.qy,
                 update.qz);
        Serial.println(msg);
        bus->receive(CommentMessage(msg));
        update.hasOrientation = false;
      }
    }

    // Invalidate insane acceleration data
    if (update.hasAcceleration) {
      if (isnan(update.ax) || isinf(update.ax) || update.ax < -1000 || update.ax > 1000 ||
          isnan(update.ay) || isinf(update.ay) || update.ay < -1000 || update.ay > 1000 ||
          isnan(update.az) || isinf(update.az) || update.az < -1000 || update.az > 1000) {
        char msg[100];
        snprintf(msg, sizeof(msg),
                 "ICM20948 invalid acceleration X=%X; Y=%X; Z=%X (%.2f, %.2f, %.2f)",
                 data.Raw_Accel.Data.X, data.Raw_Accel.Data.Y, data.Raw_Accel.Data.Z, update.ax,
                 update.ay, update.az);
        Serial.println(msg);
        bus->receive(CommentMessage(msg));
        update.hasAcceleration = false;
      }
    }

    // Only dispatch to bus if sanity checks pass
    if (update.hasOrientation || update.hasAcceleration) {
      bus->receive(update);
    }
  }
}
