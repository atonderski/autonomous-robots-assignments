/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SINGLE_TRACK_MODEL
#define SINGLE_TRACK_MODEL

#include <mutex>

#include "opendlv-standard-message-set.hpp"

class DifferentialSteeringModel {
 private:
  DifferentialSteeringModel(DifferentialSteeringModel const &) = delete;
  DifferentialSteeringModel(DifferentialSteeringModel &&) = delete;
  DifferentialSteeringModel &operator=(DifferentialSteeringModel const &) = delete;
  DifferentialSteeringModel &operator=(DifferentialSteeringModel &&) = delete;

 public:
  DifferentialSteeringModel() noexcept;
  ~DifferentialSteeringModel() = default;

 public:
  void setLeftWheelSpeed(opendlv::proxy::WheelSpeedRequest const &) noexcept;
  void setRightWheelSpeed(opendlv::proxy::WheelSpeedRequest const &) noexcept;
  opendlv::sim::KinematicState step(double) noexcept;

 private:
  std::mutex m_LeftWheelSpeedMutex;
  std::mutex m_RightWheelSpeedMutex;
  float m_LeftWheelSpeed;
  float m_RightWheelSpeed;
  float m_currentYaw;
};

#endif
