#pragma once
#include <cstdint>

namespace SonarSampling {
constexpr uint32_t QUIET_INTERVAL_MS = 25;
constexpr uint32_t ECHO_TIMEOUT_US = 20000;
// Each module is triggered at most once per complete three-sensor cycle.
constexpr uint32_t FRAME_BUDGET_MS = 3 * (ECHO_TIMEOUT_US / 1000 + QUIET_INTERVAL_MS);
// Timestamp is the OLDEST acquisition, not publication. Just before the next
// frame arrives its age can reach 110ms (acquire F/L/R) +135ms (next cycle).
// 270ms covers two bounded cycles; a stopped sampler still invalidates data.
constexpr uint32_t MAX_SAMPLE_AGE_MS = 2 * FRAME_BUDGET_MS;
static_assert(MAX_SAMPLE_AGE_MS <= 300, "Revalidate braking reserve if sampling becomes slower");
} // namespace SonarSampling
