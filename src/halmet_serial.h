#ifndef HALMET_SRC_HALMET_SERIAL_H_
#define HALMET_SRC_HALMET_SERIAL_H_

#include <esp_mac.h>

#include <cstdint>

uint64_t GetBoardSerialNumber() {
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);
  return ((uint64_t)chipid[0] << 0) + ((uint64_t)chipid[1] << 8) +
         ((uint64_t)chipid[2] << 16) + ((uint64_t)chipid[3] << 24) +
         ((uint64_t)chipid[4] << 32) + ((uint64_t)chipid[5] << 40);
}

#endif  // HALMET_SRC_HALMET_SERIAL_H_
