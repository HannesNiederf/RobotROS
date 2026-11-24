// Host tests of firmware/lib/core against the shared vectors (spez_firmware_protokoll.md section 11.1).
// Build: cmake -S firmware/test/host -B build/fw && cmake --build build/fw && ctest --test-dir build/fw
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "protocol.h"
#include "safety.h"

namespace check {
int failures = 0;
int checks = 0;
}  // namespace check

namespace core {
struct SafetyFsmTestAccess {
  static void SetEstopLevel(SafetyFsm* fsm, bool pressed) { fsm->estop_pressed_ = pressed; }
};
}  // namespace core

using core::Err;
using core::FeedResult;

namespace {

std::vector<std::string> Split(const std::string& line, char sep) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string part;
  while (std::getline(ss, part, sep)) out.push_back(part);
  if (!line.empty() && line.back() == sep) out.emplace_back();
  return out;
}

std::vector<std::vector<std::string>> ReadTable(const std::string& path, char sep) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "cannot open %s\n", path.c_str());
    ++check::failures;
  }
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    rows.push_back(Split(line, sep));
  }
  return rows;
}

// Feed a whole line plus '\n' and return the last result.
FeedResult FeedLine(core::Parser& p, const std::string& frame) {
  for (char c : frame) p.Feed(c);
  return p.Feed('\n');
}

// ------------------------------------------------------------------ protocol
void TestVectors(const std::string& dir) {
  auto rows = ReadTable(dir + "/protocol_vectors.txt", '\t');
  CHECK(rows.size() >= 40);
  int valid = 0, invalid = 0;
  for (const auto& row : rows) {
    const std::string& kind = row[0];
    const std::string& frame = row[1];
    core::Parser parser;
    FeedResult r = FeedLine(parser, frame);
    if (kind == "V" || kind == "X") {
      CHECK_MSG(r == FeedResult::kFrame, frame);
      if (r != FeedResult::kFrame) continue;
      const core::Frame& f = parser.frame();
      CHECK_MSG(row[2] == f.name, frame);
      for (size_t i = 3; i < row.size(); ++i) {
        size_t idx = i - 3;
        CHECK_MSG(idx < f.argc && row[i] == f.argv[idx], frame + " field " + std::to_string(idx));
      }
      if (kind == "V") {
        CHECK_MSG(f.argc == row.size() - 3, frame);
        // Re-encode the payload and compare with the vector (checksum generated independently).
        std::string payload = row[2];
        for (size_t i = 3; i < row.size(); ++i) payload += "," + row[i];
        char out[core::kMaxFrameLen + 1];
        size_t n = core::FormatFrame(payload.c_str(), out, sizeof(out));
        CHECK_MSG(n == frame.size() + 1 && frame + "\n" == out, frame);
        ++valid;
      }
    } else if (kind == "I") {
      CHECK_MSG(r == FeedResult::kError, frame);
      CHECK_MSG(static_cast<int>(parser.error()) == std::stoi(row[2]),
                frame + " got " + std::to_string(static_cast<int>(parser.error())));
      ++invalid;
    }
  }
  CHECK(valid >= 26);
  CHECK(invalid >= 13);
}

void TestStream() {
  core::Parser p;
  std::string data = "noise$PING,7*0B\r\n$WHO*50\n$VEL,1,2$PING,7*0B\n";
  std::vector<std::string> names;
  for (char c : data) {
    if (p.Feed(c) == FeedResult::kFrame) names.push_back(p.frame().name);
  }
  CHECK(names.size() == 3);
  CHECK(names.size() == 3 && names[0] == "PING" && names[1] == "WHO" && names[2] == "PING");

  // Overflow, then recovery.
  core::Parser q;
  std::string longline = "$" + std::string(200, 'A');
  CHECK(FeedLine(q, longline) == FeedResult::kError && q.error() == Err::kLineTooLong);
  CHECK(FeedLine(q, "$PING,7*0B") == FeedResult::kFrame);

  // Checksum error names the command for "$ERR,1,<cmd>".
  core::Parser r;
  CHECK(FeedLine(r, "$VEL,12,300,-300*00") == FeedResult::kError);
  CHECK(r.error() == Err::kChecksum && std::strcmp(r.error_command(), "VEL") == 0);
  CHECK(FeedLine(r, "$VEL,12,300,-300") == FeedResult::kError && r.error_command()[0] == '\0');

  core::Parser s;
  CHECK(FeedLine(s, "$VEL,12,300,-300*5D") == FeedResult::kFrame);
  CHECK(s.frame().msg == core::Msg::VEL && s.frame().Int(2) == -300 && s.frame().Int(0) == 12);
}

void TestWriter() {
  char payload[64];
  core::Writer w(payload, sizeof(payload));
  w.Str("ODO").Int(4294967295LL).Int(-2147483648LL).Int(0).Int(-1).Int(2147483647LL);
  CHECK(w.ok());
  CHECK(std::string(payload) == "ODO,4294967295,-2147483648,0,-1,2147483647");
  char frame[core::kMaxFrameLen + 1];
  CHECK(core::FormatFrame("WHO", frame, sizeof(frame)) == 8 && std::string(frame) == "$WHO*50\n");
  core::Writer small(payload, 4);
  small.Str("HELLO");
  CHECK(!small.ok());
  core::Writer bad(payload, sizeof(payload));
  bad.Str("CFG").Str("A,B");
  CHECK(!bad.ok());
  std::string too_long(92, 'K');
  CHECK(core::FormatFrame(too_long.c_str(), frame, sizeof(frame)) == 0);  // 92 + 5 = 97 > 96
  CHECK(core::FormatFrame(std::string(91, 'K').c_str(), frame, sizeof(frame)) == 96);
}

void TestParseInt() {
  int32_t v = 0;
  CHECK(core::ParseInt32("-2147483648", &v) && v == INT32_MIN);
  CHECK(!core::ParseInt32("2147483648", &v));
  CHECK(!core::ParseInt32("+1", &v));
  CHECK(!core::ParseInt32("", &v));
  CHECK(!core::ParseInt32("-", &v));
  CHECK(!core::ParseInt32("1.5", &v));
  CHECK(!core::ParseInt32("12 ", &v));
}

// -------------------------------------------------------------------- safety
core::SafetyState StateFromName(const std::string& s) {
  if (s == "ESTOP") return core::SafetyState::kEstop;
  if (s == "WAIT_ACK") return core::SafetyState::kWaitAck;
  if (s == "ARMED") return core::SafetyState::kArmed;
  return core::SafetyState::kBootLocked;
}

uint16_t ActionBit(const std::string& a) {
  if (a == "HELLO") return core::kActHello;
  if (a == "ERR5") return core::kActErr5;
  if (a == "ACK_ARM") return core::kActAckArm;
  if (a == "BRAKE") return core::kActBrake;
  if (a == "SETPOINTS_ZERO") return core::kActSetpointsZero;
  if (a == "DRIVE_DELAY_200MS") return core::kActDriveDelay;
  if (a == "SW_STOP_FLAG") return core::kActSwStopFlag;
  if (a == "EVT10") return core::kActEvt10;
  if (a == "EVT11") return core::kActEvt11;
  if (a == "EVT50") return core::kActEvt50;
  return 0;
}

void TestSafetyTable(const std::string& dir) {
  auto rows = ReadTable(dir + "/safety_fsm.csv", ',');
  CHECK(rows.size() == 12);  // header + 11 rows
  for (size_t i = 1; i < rows.size(); ++i) {
    const auto& r = rows[i];  // row,state,event,estop_pressed,next_state,arming,actions
    const std::string& state = r[1];
    const std::string& event = r[2];
    bool pressed = r[3] == "1";
    core::SafetyFsm fsm;
    uint16_t got = 0;
    if (event == "RESET") {
      got = fsm.Reset(1000, pressed);
    } else {
      bool before = (event == "ESTOP_PRESS" || event == "ESTOP_RELEASE") ? !pressed : pressed;
      if (state == "ESTOP") {
        fsm.Reset(0, true);
      } else {
        fsm.Reset(0, false);
        if (state == "ARMED" || state == "WAIT_ACK") fsm.ArmRequest(0, true);
        if (state == "WAIT_ACK") fsm.ArmRequest(0, false);
      }
      core::SafetyFsmTestAccess::SetEstopLevel(&fsm, before);  // row 4: input level at the event
      if (event == "ARM1") got = fsm.ArmRequest(1000, true);
      if (event == "ARM0") got = fsm.ArmRequest(1000, false);
      if (event == "ESTOP_PRESS") got = fsm.SetEstop(1000, true);
      if (event == "ESTOP_RELEASE") got = fsm.SetEstop(1000, false);
      if (event == "WD_HARD") got = fsm.Tick(fsm.last_valid_ms() + fsm.wd_hard_ms()) & ~core::kActBrake;
    }
    uint16_t expected = 0;
    for (const auto& a : Split(r[6], ';')) expected |= ActionBit(a);
    CHECK_MSG(fsm.state() == StateFromName(r[4]), "row " + r[0] + " state");
    CHECK_MSG(fsm.arming() == (r[5] == "1"), "row " + r[0] + " arming");
    if (expected & core::kActSwStopFlag) CHECK_MSG(fsm.software_stop(), "row " + r[0]);
    CHECK_MSG(got == expected, "row " + r[0] + " actions " + std::to_string(got) + " != " + std::to_string(expected));
  }
}

void TestWatchdogs() {
  core::SafetyFsm fsm;
  fsm.Reset(0, false);
  fsm.ArmRequest(0, true);
  CHECK(!fsm.DriveAllowed(199) && fsm.DriveAllowed(200));
  CHECK(fsm.Tick(499) == 0);
  CHECK(fsm.Tick(500) == (core::kActBrake | core::kActEvt50) && fsm.pi_link_lost());
  CHECK(fsm.state() == core::SafetyState::kArmed && !fsm.DriveAllowed(600));
  CHECK(fsm.ValidPiMessage(600) == core::kActEvt51 && fsm.DriveAllowed(600));
  fsm.Tick(5599);
  CHECK(fsm.state() == core::SafetyState::kArmed);
  fsm.Tick(5600);
  CHECK(fsm.state() == core::SafetyState::kWaitAck && !fsm.arming());

  // millis() overflow: watchdog arithmetic must stay correct.
  core::SafetyFsm wrap;
  wrap.Reset(0xFFFFFF00u, false);
  wrap.ArmRequest(0xFFFFFF00u, true);
  CHECK(wrap.Tick(0x00000010u) == 0);  // only 272 ms later
  CHECK(wrap.DriveAllowed(0x00000010u));
  CHECK(wrap.Tick(0xFFFFFF00u + 600u) == (core::kActBrake | core::kActEvt50));

  core::SafetyFsm locked;
  locked.Reset(0, false);
  CHECK(locked.Tick(100000) == 0 && locked.state() == core::SafetyState::kBootLocked);
  core::SafetyFsm estop;
  estop.Reset(0, true);
  CHECK(estop.ArmRequest(1, true) == core::kActErr5 && estop.state() == core::SafetyState::kEstop);
}

void TestDirectionLockAndUs() {
  struct Case {
    int32_t l, r;
    bool fwd, back;
    int32_t el, er;
  } cases[] = {
      {100, 100, true, false, 0, 0},      {-100, -100, true, false, -100, -100},     {-100, 100, true, true, -100, 100},
      {50, 150, true, false, -50, 50},    {-150, -50, false, true, -50, 50},         {0, 3, true, false, -1, 1},
      {100, 100, false, false, 100, 100}, {INT32_MAX, INT32_MAX, true, false, 0, 0},
  };
  for (const auto& c : cases) {
    int32_t l = c.l, r = c.r;
    core::ApplyDirectionLock(&l, &r, c.fwd, c.back);
    CHECK_MSG(l == c.el && r == c.er, std::to_string(c.l) + "," + std::to_string(c.r));
  }
  using core::UsClass;
  CHECK(core::ClassifyUs(65535, 120, 40, 40) == UsClass::kCliff);
  CHECK(core::ClassifyUs(161, 120, 40, 40) == UsClass::kCliff);
  CHECK(core::ClassifyUs(160, 120, 40, 40) == UsClass::kFloor);
  CHECK(core::ClassifyUs(80, 120, 40, 40) == UsClass::kFloor);
  CHECK(core::ClassifyUs(79, 120, 40, 40) == UsClass::kObstacle);
  CHECK(core::ClassifyUs(0, 20, 40, 40) == UsClass::kFloor);  // floor - margin < 0: no underflow, same as Python

  core::CliffDebounce d;
  CHECK(!d.Update(UsClass::kCliff));
  CHECK(d.Update(UsClass::kCliff));
  CHECK(d.Update(UsClass::kFloor) && d.Update(UsClass::kFloor) && !d.Update(UsClass::kFloor));
  core::CliffDebounce e;
  e.Update(UsClass::kCliff);
  e.Update(UsClass::kFloor);
  CHECK(!e.Update(UsClass::kCliff));
}

}  // namespace

int main(int argc, char** argv) {
  std::string dir = argc > 1 ? argv[1] : VECTORS_DIR;
  TestVectors(dir);
  TestStream();
  TestWriter();
  TestParseInt();
  TestSafetyTable(dir);
  TestWatchdogs();
  TestDirectionLockAndUs();
  std::printf("%d checks, %d failures\n", check::checks, check::failures);
  return check::failures == 0 ? 0 : 1;
}
