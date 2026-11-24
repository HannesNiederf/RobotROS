#include "safety.h"

namespace core {

uint16_t SafetyFsm::Reset(uint32_t now_ms, bool estop_pressed) {
  estop_pressed_ = estop_pressed;
  state_ = estop_pressed ? SafetyState::kEstop : SafetyState::kBootLocked;
  arming_ = false;
  software_stop_ = false;
  pi_link_lost_ = false;
  last_valid_ms_ = now_ms;
  return kActHello;
}

uint16_t SafetyFsm::Arm(uint32_t now_ms) {
  state_ = SafetyState::kArmed;
  arming_ = true;
  software_stop_ = false;
  armed_at_ms_ = now_ms;
  last_valid_ms_ = now_ms;
  return kActAckArm | kActSetpointsZero | kActDriveDelay;
}

uint16_t SafetyFsm::ArmRequest(uint32_t now_ms, bool arm) {
  if (arm) {
    if (estop_pressed_) return kActErr5;
    if (state_ == SafetyState::kBootLocked || state_ == SafetyState::kWaitAck) return Arm(now_ms);
    return kActAckArm;  // already ARMED: no change
  }
  if (state_ == SafetyState::kArmed) {
    state_ = SafetyState::kWaitAck;
    arming_ = false;
    software_stop_ = true;
    return kActAckArm | kActBrake | kActSwStopFlag;
  }
  return kActAckArm;
}

uint16_t SafetyFsm::SetEstop(uint32_t /*now_ms*/, bool pressed) {
  if (pressed == estop_pressed_) return 0;
  estop_pressed_ = pressed;
  if (pressed) {
    SafetyState previous = state_;
    state_ = SafetyState::kEstop;
    arming_ = false;
    return previous == SafetyState::kArmed ? (kActBrake | kActEvt10) : 0;
  }
  if (state_ == SafetyState::kEstop) {
    state_ = SafetyState::kWaitAck;
    return kActEvt11;
  }
  return 0;
}

uint16_t SafetyFsm::ValidPiMessage(uint32_t now_ms) {
  last_valid_ms_ = now_ms;
  if (pi_link_lost_) {
    pi_link_lost_ = false;
    return kActEvt51;
  }
  return 0;
}

uint16_t SafetyFsm::Tick(uint32_t now_ms) {
  if (state_ != SafetyState::kArmed) return 0;
  uint32_t silent = now_ms - last_valid_ms_;  // unsigned difference: correct across millis() overflow
  if (silent >= wd_hard_ms_) {
    state_ = SafetyState::kWaitAck;
    arming_ = false;
    return kActBrake | kActEvt50;
  }
  if (silent >= wd_soft_ms_ && !pi_link_lost_) {
    pi_link_lost_ = true;
    return kActBrake | kActEvt50;
  }
  return 0;
}

bool SafetyFsm::DriveAllowed(uint32_t now_ms) const {
  return state_ == SafetyState::kArmed && static_cast<uint32_t>(now_ms - armed_at_ms_) >= kArmDriveDelayMs &&
         !pi_link_lost_;
}

uint16_t SafetyFsm::Flags(bool block_forward, bool block_backward, bool cliff_front, bool cliff_rear) const {
  uint16_t f = 0;
  if (estop_pressed_) f |= kFlagEstopPressed;
  if (arming_) f |= kFlagRelayOn;
  if (block_forward) f |= kFlagBlockForward;
  if (block_backward) f |= kFlagBlockBackward;
  if (pi_link_lost_) f |= kFlagPiLinkLost;
  if (software_stop_) f |= kFlagSoftwareStop;
  if (cliff_front) f |= kFlagCliffFront;
  if (cliff_rear) f |= kFlagCliffRear;
  return f;
}

void ApplyDirectionLock(int32_t* l_tps, int32_t* r_tps, bool block_forward, bool block_backward) {
  int64_t linear2 = static_cast<int64_t>(*l_tps) + *r_tps;  // twice the linear part
  if (!((block_forward && linear2 > 0) || (block_backward && linear2 < 0))) return;
  int64_t diff = static_cast<int64_t>(*r_tps) - *l_tps;
  int32_t rot = static_cast<int32_t>(diff / 2);  // C++ division truncates towards zero
  *l_tps = -rot;
  *r_tps = rot;
}

UsClass ClassifyUs(uint16_t distance_mm, uint16_t floor_mm, uint16_t cliff_margin_mm, uint16_t obst_margin_mm) {
  if (distance_mm == kNoEcho || static_cast<int32_t>(distance_mm) > static_cast<int32_t>(floor_mm) + cliff_margin_mm) {
    return UsClass::kCliff;
  }
  if (static_cast<int32_t>(distance_mm) < static_cast<int32_t>(floor_mm) - obst_margin_mm) return UsClass::kObstacle;
  return UsClass::kFloor;
}

bool CliffDebounce::Update(UsClass cls) {
  if (!active_) {
    count_ = cls == UsClass::kCliff ? count_ + 1 : 0;
    if (count_ >= on_) {
      active_ = true;
      count_ = 0;
    }
  } else {
    count_ = cls == UsClass::kFloor ? count_ + 1 : 0;
    if (count_ >= off_) {
      active_ = false;
      count_ = 0;
    }
  }
  return active_;
}

}  // namespace core
