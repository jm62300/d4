/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <algorithm>
#include <chrono>

namespace bipe {
class Timer {
 private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration =
      std::chrono::duration<double>;  // Stores time in seconds as a double

  Duration m_timeLimit;
  Duration m_elapsedTime;
  TimePoint m_lastStartTime;
  bool m_isRunning;

 public:
  /**
   * @brief Constructs a Timer with a specific time limit.
   * @param limitInSeconds The time limit in seconds.
   */
  explicit Timer(double limitInSeconds)
      : m_timeLimit(limitInSeconds),
        m_elapsedTime(Duration::zero()),
        m_isRunning(false) {}

  /**
   * @brief Starts or resumes the timer.
   */
  void start() {
    if (!m_isRunning && !isTimeout()) {
      m_lastStartTime = Clock::now();
      m_isRunning = true;
    }
  }

  /**
   * @brief Pauses the timer, preserving the elapsed time.
   */
  void pause() {
    if (m_isRunning) {
      m_elapsedTime += (Clock::now() - m_lastStartTime);
      m_isRunning = false;
    }
  }

  /**
   * @brief Calculates how much time is left.
   * @return Remaining time in seconds. Returns 0.0 if the limit is reached.
   */
  double getRemainingTime() const {
    Duration totalElapsed = m_elapsedTime;

    // If it's currently running, add the time since we last called start()
    if (m_isRunning) {
      totalElapsed += (Clock::now() - m_lastStartTime);
    }

    Duration remaining = m_timeLimit - totalElapsed;

    // Prevent returning negative numbers if we overshot the limit
    return std::max(0.0, remaining.count());
  }

  /**
   * @brief Checks if the time limit has been reached.
   * @return True if timeout is reached, false otherwise.
   */
  bool isTimeout() const { return m_isRunning && (getRemainingTime() <= 0.0); }

  /**
   * @brief (Optional utility) Resets the timer back to zero.
   */
  void reset() {
    m_elapsedTime = Duration::zero();
    m_isRunning = false;
  }
};
}  // namespace bipe