// Minimal test helpers for the firmware host tests (no external framework needed).
// PlatformIO's Unity can replace this later (spec section 11.1); the test cases stay the same.
#pragma once

#include <cstdio>
#include <string>

namespace check {
extern int failures;
extern int checks;
}  // namespace check

#define CHECK(cond)                                                                 \
  do {                                                                              \
    ++check::checks;                                                                \
    if (!(cond)) {                                                                  \
      ++check::failures;                                                            \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    }                                                                               \
  } while (0)

#define CHECK_MSG(cond, msg)                                                                                       \
  do {                                                                                                             \
    ++check::checks;                                                                                               \
    if (!(cond)) {                                                                                                 \
      ++check::failures;                                                                                           \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s [%s]\n", __FILE__, __LINE__, #cond, std::string(msg).c_str()); \
    }                                                                                                              \
  } while (0)
