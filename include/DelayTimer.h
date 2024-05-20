#pragma once

class DelayTimer {
 public:
  DelayTimer(const unsigned long duration)
      : m_delayDuration(duration), m_timestamp(millis()) {}

  void reset() { m_timestamp = millis(); }

  bool isExpired() const {
    unsigned long elapsed_time = millis() - m_timestamp;
    return elapsed_time > m_delayDuration;
  }

 private:
  unsigned long m_timestamp;
  unsigned long m_delayDuration;
};