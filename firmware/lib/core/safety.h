// Safety state machine of the DRIVE role and related pure logic
// (docs/spec/spez_firmware_protokoll.md sections 5 and 6.1). Same behaviour as
// ros2_ws/src/robotros_bridge/robotros_bridge/core/safety.py; both are tested against
// firmware/test/vectors/safety_fsm.csv. Times are monotonic milliseconds from the caller.
#pragma once

#include <stdint.h>

namespace core {

enum class SafetyState : uint8_t { kBootLocked = 0, kEstop = 1, kWaitAck = 2, kArmed = 3 };

// Bits of the $SAF flags field (section 2.3).
enum SafFlag : uint16_t {
  kFlagEstopPressed = 1u << 0,
  kFlagRelayOn = 1u << 1,
  kFlagBlockForward = 1u << 2,
  kFlagBlockBackward = 1u << 3,
  kFlagPiLinkLost = 1u << 4,
  kFlagSoftwareStop = 1u << 5,
  kFlagCliffFront = 1u << 6,
  kFlagCliffRear = 1u << 7,
  kFlagHubFault = 1u << 8,
};

// Actions the caller has to carry out after an event (bit set).
enum SafetyAction : uint16_t {
  kActHello = 1u << 0,
  kActErr5 = 1u << 1,
  kActAckArm = 1u << 2,
  kActBrake = 1u << 3,
  kActSetpointsZero = 1u << 4,
  kActDriveDelay = 1u << 5,
  kActSwStopFlag = 1u << 6,
  kActEvt10 = 1u << 7,   // E-stop pressed
  kActEvt11 = 1u << 8,   // E-stop released
  kActEvt50 = 1u << 9,   // Pi link lost
  kActEvt51 = 1u << 10,  // Pi link back
};

constexpr uint32_t kArmDriveDelayMs = 200;

class SafetyFsm {
 public:
  SafetyFsm(uint32_t wd_soft_ms = 500, uint32_t wd_hard_ms = 5000) : wd_soft_ms_(wd_soft_ms), wd_hard_ms_(wd_hard_ms) {}

  void set_watchdogs(uint32_t soft_ms, uint32_t hard_ms) {
    wd_soft_ms_ = soft_ms;
    wd_hard_ms_ = hard_ms;
  }

  uint16_t Reset(uint32_t now_ms, bool estop_pressed);  // rows 1-2
  uint16_t ArmRequest(uint32_t now_ms, bool arm);       // rows 3, 4, 8, 10
  uint16_t SetEstop(uint32_t now_ms, bool pressed);     // rows 5, 6, 7, 9 (input already debounced)
  uint16_t ValidPiMessage(uint32_t now_ms);             // VEL or PING with correct checksum
  uint16_t Tick(uint32_t now_ms);                       // watchdogs, row 11
  bool DriveAllowed(uint32_t now_ms) const;

  SafetyState state() const { return state_; }
  bool arming() const { return arming_; }
  bool estop_pressed() const { return estop_pressed_; }
  bool software_stop() const { return software_stop_; }
  bool pi_link_lost() const { return pi_link_lost_; }
  uint32_t last_valid_ms() const { return last_valid_ms_; }
  uint32_t wd_hard_ms() const { return wd_hard_ms_; }
  uint16_t Flags(bool block_forward, bool block_backward, bool cliff_front, bool cliff_rear) const;

 private:
  friend struct SafetyFsmTestAccess;  // host tests only: set the E-stop input level without an event
  uint16_t Arm(uint32_t now_ms);
  uint32_t wd_soft_ms_;
  uint32_t wd_hard_ms_;
  SafetyState state_ = SafetyState::kBootLocked;
  bool estop_pressed_ = false;
  bool arming_ = false;
  bool software_stop_ = false;
  bool pi_link_lost_ = false;
  uint32_t last_valid_ms_ = 0;
  uint32_t armed_at_ms_ = 0;
};

// Clamp the linear part of a wheel setpoint pair; rotation in place stays allowed (section 5.3).
void ApplyDirectionLock(int32_t* l_tps, int32_t* r_tps, bool block_forward, bool block_backward);

enum class UsClass : uint8_t { kUnknown = 0, kFloor = 1, kCliff = 2, kObstacle = 3 };
constexpr uint16_t kNoEcho = 65535;

UsClass ClassifyUs(uint16_t distance_mm, uint16_t floor_mm, uint16_t cliff_margin_mm, uint16_t obst_margin_mm);

// Cliff confirmation: on after `confirm_on` cliff readings in a row, off after `confirm_off` floor readings.
class CliffDebounce {
 public:
  CliffDebounce(uint8_t confirm_on = 2, uint8_t confirm_off = 3) : on_(confirm_on), off_(confirm_off) {}
  void set_counts(uint8_t confirm_on, uint8_t confirm_off) {
    on_ = confirm_on;
    off_ = confirm_off;
  }
  bool Update(UsClass cls);
  bool active() const { return active_; }

 private:
  uint8_t on_;
  uint8_t off_;
  uint8_t count_ = 0;
  bool active_ = false;
};

}  // namespace core
