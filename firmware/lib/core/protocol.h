// Serial protocol version 1 between Pi and Nano (docs/spec/spez_firmware_protokoll.md section 2).
// Host-testable core: no Arduino headers, no heap, no exceptions. Same behaviour as
// ros2_ws/src/robotros_bridge/robotros_bridge/core/protocol.py; both are tested against
// firmware/test/vectors/protocol_vectors.{json,txt}.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace core {

constexpr uint8_t kProtoVer = 1;
constexpr size_t kMaxFrameLen = 96;  // bytes including '$', '*', checksum and '\n'
constexpr uint8_t kMaxFields = 8;    // command name plus up to 7 fields (LED has 6)

enum class Err : uint8_t {
  kNone = 0,
  kChecksum = 1,
  kUnknownCommand = 2,
  kBadArgument = 3,
  kRoleNotSupported = 4,
  kNotAllowedInState = 5,
  kLineTooLong = 6,
  kBusy = 7,
};

enum class Msg : uint8_t {
  // Pi -> Nano
  WHO,
  PING,
  VEL,
  ARM,
  CLR,
  CFG,
  HUB,
  LED,
  BUZ,
  REL,
  // Nano -> Pi
  HELLO,
  PONG,
  ODO,
  SEN,
  SAF,
  PWR,
  HUBST,
  EVT,
  ACK,
  ERR,
  kCount,
  kUnknown = 0xFF,
};

uint8_t Checksum(const char* payload, size_t len);

// Strict decimal integer: optional '-', at least one digit, no '+', no spaces. False on overflow.
bool ParseInt32(const char* text, int32_t* out);
bool ParseInt64(const char* text, int64_t* out);

const char* MsgName(Msg msg);
Msg MsgFromName(const char* name);

// A decoded frame. Field texts point into the parser's buffer and stay valid until the next Feed().
struct Frame {
  Msg msg = Msg::kUnknown;
  const char* name = nullptr;
  uint8_t argc = 0;  // number of fields after the name, including ignored extra fields
  const char* argv[kMaxFields] = {};
  // Integer value of field i (only meaningful for integer fields that passed validation).
  int64_t Int(uint8_t i) const;
};

// Result of feeding one byte.
enum class FeedResult : uint8_t { kNone, kFrame, kError };

// Incremental parser. '\r' is ignored, '$' starts (or restarts) a frame, '\n' ends it, bytes outside
// a frame are dropped. Frames longer than kMaxFrameLen give Err::kLineTooLong.
class Parser {
 public:
  FeedResult Feed(char c);
  const Frame& frame() const { return frame_; }
  Err error() const { return error_; }
  // Command name of the last frame that failed the checksum (for "$ERR,1,<cmd>"), else "".
  const char* error_command() const { return error_command_; }

 private:
  FeedResult Finish();
  char buf_[kMaxFrameLen + 1] = {};
  size_t len_ = 0;
  bool in_frame_ = false;
  bool overflow_ = false;
  Frame frame_;
  Err error_ = Err::kNone;
  char error_command_[8] = {};
};

// Decode one complete frame text ("$...*HH", no line end) in place. Used by Parser and the tests.
// `text` is modified (separators become '\0'). Returns Err::kNone on success.
Err DecodeInPlace(char* text, size_t len, Frame* out, char* error_command, size_t error_command_cap);

// Write "$<payload>*HH\n" into out. Returns the number of bytes written, 0 if it does not fit or the
// frame would exceed kMaxFrameLen.
size_t FormatFrame(const char* payload, char* out, size_t cap);

// Small helper to build payloads without heap or printf: Writer w(buf, cap); w.Str("ODO").Int(t)...
class Writer {
 public:
  Writer(char* buf, size_t cap);
  Writer& Str(const char* s);  // first call: the command name; later calls: a string field
  Writer& Int(int64_t value);  // integer field
  bool ok() const { return ok_; }
  const char* c_str() const { return buf_; }
  size_t size() const { return len_; }

 private:
  void Put(char c);
  void Sep();
  char* buf_;
  size_t cap_;
  size_t len_ = 0;
  bool first_ = true;
  bool ok_ = true;
};

}  // namespace core
