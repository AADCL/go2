#pragma once
#include <cstdint>

namespace go2_control {
struct ClassicWalkResult {
  int32_t zero_result = -1;
  int32_t classic_result = -1;
  bool accepted() const { return zero_result == 0 && classic_result == 0; }
};

// Run only while disarmed. Never substitutes controller selection, StandUp,
// legacy SwitchGait, or an unacknowledged request for ClassicWalk(true).
template <typename SportClient>
ClassicWalkResult requestClassicWalk(SportClient& client) {
  ClassicWalkResult result;
  result.zero_result = client.Move(0.0f, 0.0f, 0.0f);
  if (result.zero_result == 0) result.classic_result = client.ClassicWalk(true);
  return result;
}

inline bool changesSportMode(int64_t api) {
  switch (api) {
    case 1001: case 1002: case 1003: case 1004: case 1005: case 1006:
    case 1007: case 1009: case 1010: case 1011: case 1016: case 1017:
    case 1019: case 1020: case 1022: case 1023: case 1027: case 1028:
    case 1029: case 1030: case 1031: case 1032: case 1036:
    case 1061: case 1062: case 1063: case 2041: case 2043: case 2044:
    case 2045: case 2046: case 2047: case 2048: case 2049: case 2050:
    case 2051: case 2054: case 2058:
      return true;
    default: return false;
  }
}
}  // namespace go2_control
