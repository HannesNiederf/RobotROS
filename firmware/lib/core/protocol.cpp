#include "protocol.h"

#include <string.h>

namespace core {
namespace {

struct FieldSpec {
  bool is_int;
  int64_t lo;
  int64_t hi;
};

constexpr int64_t kU8 = 255;
constexpr int64_t kU16 = 65535;
constexpr int64_t kU32 = 4294967295LL;
constexpr int64_t kI32Min = -2147483648LL;
constexpr int64_t kI32Max = 2147483647LL;

constexpr FieldSpec I(int64_t lo, int64_t hi) { return FieldSpec{true, lo, hi}; }
constexpr FieldSpec S() { return FieldSpec{false, 0, 0}; }

struct Schema {
  const char* name;
  uint8_t count;     // number of fields
  uint8_t optional;  // trailing fields that may be missing
  FieldSpec f[7];
};

// Same order as enum Msg. Ranges as in protocol.py (spec section 2.2 and 2.3).
const Schema kSchemas[] = {
    {"WHO", 0, 0, {}},
    {"PING", 1, 0, {I(0, kU32)}},
    {"VEL", 3, 0, {I(0, kU32), I(kI32Min, kI32Max), I(kI32Min, kI32Max)}},
    {"ARM", 1, 0, {I(0, 1)}},
    {"CLR", 0, 0, {}},
    {"CFG", 2, 0, {S(), I(kI32Min, kI32Max)}},
    {"HUB", 2, 0, {I(0, 2), I(0, kU8)}},
    {"LED", 5, 0, {I(0, 2), I(0, 3), I(0, kU8), I(0, kU8), I(0, kU8)}},
    {"BUZ", 1, 0, {I(0, 3)}},
    {"REL", 2, 0, {I(1, 3), I(0, 1)}},
    {"HELLO", 4, 0, {I(0, kU16), S(), I(1, 3), I(0, kU8)}},
    {"PONG", 1, 0, {I(0, kU32)}},
    {"ODO", 5, 0, {I(0, kU32), I(kI32Min, kI32Max), I(kI32Min, kI32Max), I(kI32Min, kI32Max), I(kI32Min, kI32Max)}},
    {"SEN", 6, 0, {I(0, kU32), I(0, kU16), I(0, kU16), I(0, 3), I(0, 3), I(0, 7)}},
    {"SAF", 3, 0, {I(0, kU32), I(0, 3), I(0, kU16)}},
    {"PWR", 5, 0, {I(0, kU32), I(0, kU16), I(0, kU16), I(0, kU16), I(-32768, 32767)}},
    {"HUBST", 4, 0, {I(0, kU32), I(0, 5), I(0, 3), I(0, 2)}},
    {"EVT", 3, 0, {I(0, kU32), I(0, kU8), I(kI32Min, kI32Max)}},
    {"ACK", 2, 1, {S(), S()}},
    {"ERR", 2, 1, {I(1, 7), S()}},
};
static_assert(sizeof(kSchemas) / sizeof(kSchemas[0]) == static_cast<size_t>(Msg::kCount), "schema table");

bool IsPayloadChar(char c) {
  // Printable ASCII without space; '$' and '*' are framing characters.
  return c >= 0x21 && c <= 0x7E && c != '$' && c != '*';
}

int HexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;  // lower case is not allowed (spec: upper-case hex digits)
}

void CopyName(const char* src, size_t len, char* dst, size_t cap) {
  if (cap == 0) return;
  size_t n = len < cap - 1 ? len : cap - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

}  // namespace

uint8_t Checksum(const char* payload, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) sum ^= static_cast<uint8_t>(payload[i]);
  return sum;
}

bool ParseInt64(const char* text, int64_t* out) {
  const char* p = text;
  bool neg = false;
  if (*p == '-') {
    neg = true;
    ++p;
  }
  if (*p == '\0') return false;
  // 19 digits fit in int64; our ranges need at most 10, so reject anything longer than 18.
  uint64_t value = 0;
  size_t digits = 0;
  for (; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    if (++digits > 18) return false;
    value = value * 10 + static_cast<uint64_t>(*p - '0');
  }
  *out = neg ? -static_cast<int64_t>(value) : static_cast<int64_t>(value);
  return true;
}

bool ParseInt32(const char* text, int32_t* out) {
  int64_t v;
  if (!ParseInt64(text, &v) || v < kI32Min || v > kI32Max) return false;
  *out = static_cast<int32_t>(v);
  return true;
}

const char* MsgName(Msg msg) {
  size_t i = static_cast<size_t>(msg);
  return i < static_cast<size_t>(Msg::kCount) ? kSchemas[i].name : "";
}

Msg MsgFromName(const char* name) {
  for (size_t i = 0; i < static_cast<size_t>(Msg::kCount); ++i) {
    if (strcmp(kSchemas[i].name, name) == 0) return static_cast<Msg>(i);
  }
  return Msg::kUnknown;
}

int64_t Frame::Int(uint8_t i) const {
  int64_t v = 0;
  if (i < argc) ParseInt64(argv[i], &v);
  return v;
}

Err DecodeInPlace(char* text, size_t len, Frame* out, char* error_command, size_t error_command_cap) {
  if (error_command_cap) error_command[0] = '\0';
  *out = Frame();
  if (len + 1 > kMaxFrameLen) return Err::kLineTooLong;
  if (len == 0 || text[0] != '$') return Err::kBadArgument;
  // Checksum: last '*' must be followed by exactly two upper-case hex digits.
  size_t star = len;
  for (size_t i = len; i-- > 1;) {
    if (text[i] == '*') {
      star = i;
      break;
    }
  }
  char* payload = text + 1;
  size_t payload_len = star == len ? 0 : star - 1;
  size_t name_len = 0;
  while (name_len < payload_len && payload[name_len] != ',') ++name_len;
  if (star == len || len - star != 3) return Err::kChecksum;
  CopyName(payload, name_len, error_command, error_command_cap);
  int hi = HexValue(text[star + 1]);
  int lo = HexValue(text[star + 2]);
  if (hi < 0 || lo < 0) return Err::kChecksum;
  if (static_cast<uint8_t>(hi * 16 + lo) != Checksum(payload, payload_len)) return Err::kChecksum;
  if (error_command_cap) error_command[0] = '\0';  // only checksum errors name the command
  for (size_t i = 0; i < payload_len; ++i) {
    if (payload[i] != ',' && !IsPayloadChar(payload[i])) return Err::kBadArgument;
  }
  if (name_len == 0) return Err::kUnknownCommand;
  // Split in place.
  payload[payload_len] = '\0';
  uint8_t count = 0;
  const char* parts[kMaxFields + 8];
  parts[count++] = payload;
  for (size_t i = 0; i < payload_len; ++i) {
    if (payload[i] == ',') {
      payload[i] = '\0';
      if (count < sizeof(parts) / sizeof(parts[0])) parts[count++] = payload + i + 1;
    }
  }
  Msg msg = MsgFromName(parts[0]);
  if (msg == Msg::kUnknown) return Err::kUnknownCommand;
  const Schema& sc = kSchemas[static_cast<size_t>(msg)];
  uint8_t args = count - 1;
  if (args < sc.count - sc.optional) return Err::kBadArgument;
  for (uint8_t i = 0; i < sc.count && i < args; ++i) {
    const char* field = parts[i + 1];
    if (sc.f[i].is_int) {
      int64_t v;
      if (!ParseInt64(field, &v) || v < sc.f[i].lo || v > sc.f[i].hi) return Err::kBadArgument;
    } else if (field[0] == '\0') {
      return Err::kBadArgument;
    }
  }
  out->msg = msg;
  out->name = parts[0];
  out->argc = args < kMaxFields ? args : kMaxFields;
  for (uint8_t i = 0; i < out->argc; ++i) out->argv[i] = parts[i + 1];
  return Err::kNone;
}

FeedResult Parser::Feed(char c) {
  if (c == '\r') return FeedResult::kNone;
  if (c == '$') {
    buf_[0] = '$';
    len_ = 1;
    in_frame_ = true;
    overflow_ = false;
    return FeedResult::kNone;
  }
  if (!in_frame_) return FeedResult::kNone;
  if (c == '\n') return Finish();
  if (len_ + 1 >= kMaxFrameLen) {  // +1 for the '\n' still to come
    overflow_ = true;
    return FeedResult::kNone;
  }
  buf_[len_++] = c;
  return FeedResult::kNone;
}

FeedResult Parser::Finish() {
  in_frame_ = false;
  error_command_[0] = '\0';
  if (overflow_) {
    error_ = Err::kLineTooLong;
    return FeedResult::kError;
  }
  buf_[len_] = '\0';
  error_ = DecodeInPlace(buf_, len_, &frame_, error_command_, sizeof(error_command_));
  return error_ == Err::kNone ? FeedResult::kFrame : FeedResult::kError;
}

size_t FormatFrame(const char* payload, char* out, size_t cap) {
  size_t n = strlen(payload);
  size_t total = n + 5;  // '$' + payload + '*' + 2 hex + '\n'
  if (total > kMaxFrameLen || total + 1 > cap) return 0;
  static const char kHex[] = "0123456789ABCDEF";
  uint8_t sum = Checksum(payload, n);
  out[0] = '$';
  memcpy(out + 1, payload, n);
  out[n + 1] = '*';
  out[n + 2] = kHex[sum >> 4];
  out[n + 3] = kHex[sum & 0x0F];
  out[n + 4] = '\n';
  out[n + 5] = '\0';
  return total;
}

Writer::Writer(char* buf, size_t cap) : buf_(buf), cap_(cap) {
  if (cap_)
    buf_[0] = '\0';
  else
    ok_ = false;
}

void Writer::Put(char c) {
  if (len_ + 1 >= cap_) {
    ok_ = false;
    return;
  }
  buf_[len_++] = c;
  buf_[len_] = '\0';
}

void Writer::Sep() {
  if (!first_) Put(',');
  first_ = false;
}

Writer& Writer::Str(const char* s) {
  Sep();
  for (; *s; ++s) {
    if (!IsPayloadChar(*s) || *s == ',') ok_ = false;
    Put(*s);
  }
  return *this;
}

Writer& Writer::Int(int64_t value) {
  Sep();
  char tmp[21];
  size_t n = 0;
  uint64_t mag = value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1 : static_cast<uint64_t>(value);
  do {
    tmp[n++] = static_cast<char>('0' + mag % 10);
    mag /= 10;
  } while (mag);
  if (value < 0) Put('-');
  while (n) Put(tmp[--n]);
  return *this;
}

}  // namespace core
