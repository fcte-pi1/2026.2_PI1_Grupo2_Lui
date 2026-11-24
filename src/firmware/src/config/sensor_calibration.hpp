#pragma once
namespace SensorCalibration {
// Provisional offsets: mean error at user references 3cm and 10cm,
// measured from metal transducer faces on 16/11/2026.
// Left revised using repeated 5/10cm references; checked again at 3cm.
// Earlier inconsistent placements remain documented in the calibration report.
// Hardware later identified as HC-SR04; supply/ECHO adaptation unverified.
// Offsets are provisional and have not resolved setup-dependent discrepancies.
// No gain adjustment or claim of accuracy outside the measured interval.
constexpr float FRONT_OFFSET_CM = 0.82f;
constexpr float LEFT_OFFSET_CM = 0.50f;
constexpr float RIGHT_OFFSET_CM = 0.65f;
}
