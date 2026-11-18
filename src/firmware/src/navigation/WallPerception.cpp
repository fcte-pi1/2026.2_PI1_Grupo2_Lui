#include "WallPerception.h"

#include <array>

namespace perception {

bool isPhysicallyPlausible(uint16_t rawDistanceMm) {
    return rawDistanceMm >= kUs100MinRangeMm && rawDistanceMm <= kUs100MaxRangeMm;
}

uint16_t MovingAverageFilter3::push(uint16_t rawValue) {
    buffer_[cursor_] = rawValue;
    cursor_ = static_cast<uint8_t>((cursor_ + 1) % kWindowSize);
    if (filledCount_ < kWindowSize) {
        ++filledCount_;
    }

    uint32_t sum = 0;
    for (uint8_t i = 0; i < filledCount_; ++i) {
        sum += buffer_[i];
    }
    return static_cast<uint16_t>(sum / filledCount_);
}

void applyReading(SensorReading& reading, const RawEcho& echo, MovingAverageFilter3& filter) {
    reading.timestampMs = echo.timestampMs;

    if (!echo.valid) {
        reading.valid = false;
        reading.rawDistanceMm = echo.rawDistanceMm;
        reading.filteredDistanceMm = 0;
        filter = MovingAverageFilter3{};
        return;
    }

    if (!echo.hasEcho) {
        // Sem eco = fora de alcance = "caminho livre" (Edge Cases do spec),
        // NÃO é uma leitura inválida (FR-005).
        reading.valid = true;
        reading.rawDistanceMm = kUs100MaxRangeMm;
        reading.filteredDistanceMm = filter.push(kUs100MaxRangeMm);
        return;
    }

    if (!isPhysicallyPlausible(echo.rawDistanceMm)) {
        reading.valid = false;
        reading.rawDistanceMm = echo.rawDistanceMm;
        // Break the history across an invalid acquisition: old open-space
        // values must not mask the first nearby wall after recovery.
        reading.filteredDistanceMm = 0;
        filter = MovingAverageFilter3{};
        return;
    }

    reading.valid = true;
    reading.rawDistanceMm = echo.rawDistanceMm;
    reading.filteredDistanceMm = echo.preFiltered ? echo.rawDistanceMm : filter.push(echo.rawDistanceMm);
}

void updateFaultState(SensorFaultState& fault, bool readingValid) {
    if (readingValid) {
        fault.consecutiveInvalidCount = 0;
        fault.isFaulted = false;
        return;
    }

    if (fault.consecutiveInvalidCount < 5) {
        ++fault.consecutiveInvalidCount;
    }
    if (fault.consecutiveInvalidCount >= 5) {
        fault.isFaulted = true;
    }
}

WallPerception::WallPerception(IWallSensor& sensor, uint16_t thresholdMm)
    : sensor_(sensor), thresholdMm_(thresholdMm) {}

void WallPerception::reset() {
    filters_ = {};
    readings_ = {};
    faults_ = {};
    state_ = {};
    lastProcessedTimestampMs_ = 0;
    hasProcessedSample_ = false;
}

void WallPerception::update() {
    static constexpr std::array<Direction, kDirectionCount> kDirections{
        Direction::Front, Direction::Left, Direction::Right};
    std::array<RawEcho, kDirectionCount> echoes{};
    for (Direction direction : kDirections) {
        echoes[indexOf(direction)] = sensor_.read(direction);
    }

    // HardwareWallSensor pode expor um snapshot em cache. Processar o mesmo
    // timestamp mais de uma vez inflaria artificialmente a média móvel.
    const uint32_t sampleTimestamp = echoes[indexOf(Direction::Front)].timestampMs;
    const bool repeatedSample = hasProcessedSample_ && sampleTimestamp == lastProcessedTimestampMs_;
    hasProcessedSample_ = true;
    lastProcessedTimestampMs_ = sampleTimestamp;

    for (Direction direction : kDirections) {
        const std::size_t idx = indexOf(direction);
        // A cached frame can expire without changing its timestamp. Honor
        // invalidation immediately, but do not filter/count the sample again.
        if (repeatedSample && echoes[idx].valid) continue;
        readings_[idx].direction = direction;
        applyReading(readings_[idx], echoes[idx], filters_[idx]);
        faults_[idx].direction = direction;
        if (!repeatedSample) updateFaultState(faults_[idx], readings_[idx].valid);
    }

    const SensorReading& front = readings_[indexOf(Direction::Front)];
    const SensorReading& left = readings_[indexOf(Direction::Left)];
    const SensorReading& right = readings_[indexOf(Direction::Right)];

    state_.frontValid = front.valid;
    state_.leftValid = left.valid;
    state_.rightValid = right.valid;

    // Uma amostra inválida não pode virar uma parede por causa de um valor
    // inicial zero, nem virar um caminho livre por causa de um valor antigo.
    // A validade explícita preserva a distinção entre Open e Unknown.
    state_.wallFront = front.valid && front.filteredDistanceMm <= thresholdMm_;
    state_.wallLeft = left.valid && left.filteredDistanceMm <= thresholdMm_;
    state_.wallRight = right.valid && right.filteredDistanceMm <= thresholdMm_;

    state_.distFrontMm = front.valid ? front.filteredDistanceMm : 0;
    state_.distLeftMm = left.valid ? left.filteredDistanceMm : 0;
    state_.distRightMm = right.valid ? right.filteredDistanceMm : 0;

    state_.timestampMs = front.timestampMs;
}

uint16_t WallPerception::lateralDistanceMm(Direction side) const {
    return side == Direction::Left ? state_.distLeftMm : state_.distRightMm;
}

bool WallPerception::hasFault(Direction direction) const {
    return faults_[indexOf(direction)].isFaulted;
}

}  // namespace perception
