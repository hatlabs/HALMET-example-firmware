#ifndef COUNTING_NMEA2000_H_
#define COUNTING_NMEA2000_H_

#include <NMEA2000_esp32.h>

#include "sensesp/system/observablevalue.h"

// tNMEA2000_esp32 that counts the N2K messages SendMsg accepted (returned
// true), so the TX count lives in one place (the bus) rather than being
// replicated across every sender. Accepted means every frame was handed to the
// TWAI driver or queued in the library's send buffer (sized in main.cpp).
// Not counted: sends while the address claim is still pending, or
// once that send buffer is full. A bus-off shows up here only after the buffer
// has filled.
//
// tNMEA2000::SendMsg is NOT virtual, so this *hides* it instead of overriding:
// the count only happens for callers that hold a CountingNMEA2000* (resolved by
// static type, not vtable). Senders therefore take CountingNMEA2000*, not
// tNMEA2000*. Messages the library emits on its own (ISO address claim,
// heartbeat, PGN-list/product-info responses) go through the base SendMsg and
// are not counted -- this is an application message counter, and it counts once
// per SendMsg (per PGN), not per CAN frame, so it lines up with the per-message
// counts a gateway reports rather than fast-packet frame counts.
class CountingNMEA2000 : public tNMEA2000_esp32 {
 public:
  using tNMEA2000_esp32::tNMEA2000_esp32;

  bool SendMsg(const tN2kMsg& msg, int device_index = 0) {
    bool ok = tNMEA2000::SendMsg(msg, device_index);
    if (ok) {
      tx_count_.set(tx_count_.get() + 1);
    }
    return ok;
  }

  sensesp::ObservableValue<int> tx_count_{0};
};

#endif  // COUNTING_NMEA2000_H_
