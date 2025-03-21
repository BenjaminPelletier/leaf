#include "igc.h"

#include <SD_MMC.h>

#include "Arduino.h"
#include "ArduinoJson.h"
#include "FS.h"
#include "instruments/baro.h"
#include "instruments/gps.h"
#include "time.h"
#include "ui/settings/settings.h"
#include "utils/string_utils.h"
#include "version.h"

String latDegreeToStr(double degree) {
  char output[9];  // 8 bytes + null terminator
  char hemisphere = (degree >= 0) ? 'N' : 'S';
  degree = abs(degree);

  int degrees = (int)degree;
  double minutes = (degree - degrees) * 60;
  int intMinutes = (int)minutes;
  int fractionalMinutes = (int)((minutes - intMinutes) * 1000);

  snprintf(output, 9, "%02d%02d%03d%c", degrees, intMinutes, fractionalMinutes, hemisphere);
  return String(output);
}

String lngDegreeToStr(double degree) {
  char output[10];  // 9 bytes + null terminator
  char hemisphere = (degree >= 0) ? 'E' : 'W';
  degree = abs(degree);

  int degrees = (int)degree;
  double minutes = (degree - degrees) * 60;
  int intMinutes = (int)minutes;
  int fractionalMinutes = (int)((minutes - intMinutes) * 1000);

  snprintf(output, 10, "%03d%02d%03d%c", degrees, intMinutes, fractionalMinutes, hemisphere);
  return String(output);
}

const String Igc::desiredFileName() const {
  // Name of the file should be for example 2024-12-10-XFH-000-01.IGC
  // as per the IGC spec.
  char buf[11];
  tm cal;
  gps.getLocalDateTime(cal);
  strftime(buf, 11, "%F", &cal);

  String ret = buf;
  ret += (String) "-" + IGC_MANUFACTURER_CODE + "-000";
  return ret;
}

void Igc::log(unsigned long durationSec) {
  // Short-circuit if we've not yet started a flight
  if (!started()) return;

  // Generate the time in HHMMSS
  char buf[8];
  tm cal;
  gps.getUtcDateTime(cal);
  strftime(buf, sizeof(buf), "%H%M%S", &cal);

  logger.writeBRecord(buf,  // Time in HHMMSS
                      latDegreeToStr(gps.location.lat()), lngDegreeToStr(gps.location.lng()), true,
                      baro.alt / 100,  // cm to meters
                      gps.altitude.meters(), toDigits((int)gps.fixInfo.error, 3));
}

bool Igc::startFlight() {
  auto success = Flight::startFlight();
  if (!success) return false;

  logger.setOutput(file);

  // Log the Header
  // A record to look like "AXLFLeaf1"
  logger.setManufacturerId(IGC_MANUFACTURER_CODE);
  logger.setLoggerId("Lea");
  logger.setIdExtension("f1");

  logger.pilot = "Unknown";
  logger.glider_type = "Unknown";
  // Overwrite from file if set in the Pilot descriptor
  setPilotFromFile();

  logger.firmware_version = FIRMWARE_VERSION;
  logger.hardware_version = "Leaf1";
  logger.logger_type = (String) "Leaf1," + FIRMWARE_VERSION;
  logger.gps_type = "GNSS LC86G";
  logger.pressure_type = "MS5611";
  logger.time_zone = (String)(settings.system_timeZone / 60);

  tm cal;
  gps.getUtcDateTime(cal);
  strftime(logger.date, sizeof(logger.date), "%d%m%y", &cal);

  logger.writeHeader();

  // Log the I record (saying we're going to log the, now manditory, FXA record)
  const IRecordExtension extensions[] = {IRecordExtension(3, "FXA")};
  logger.writeIRecord(sizeof(extensions) / sizeof(extensions[0]), extensions);

  return true;
}

void Igc::end(const FlightStats stats) {
  // If we've not started a flight yet, don't write to disk.
  if (started()) {
    logger.writeGRecord();
  }
  Flight::end(stats);
}

void Igc::setPilotFromFile() {
  auto pilotFile = SD_MMC.open("/pilot.json", "r");
  if (!pilotFile) {
    return;  // No pilot JSON file
  }
  JsonDocument doc;
  deserializeJson(doc, pilotFile);
  logger.pilot = (String)doc["pilot"];
  logger.glider_type = (String)doc["glider_type"];
}