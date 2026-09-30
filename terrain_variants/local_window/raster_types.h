#pragma once
#include <cstdint>

struct Record {
  int32_t ix, iy, support, reserved;
  double lo, hi;
  double xyz[24];
};

struct SpanRecord {
  int32_t ix, iy;
  int64_t lo, hi, ceiling;
  int32_t slope_ok, walkable;
};
