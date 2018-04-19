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

#include <cmath>
#include <iostream>

#include "differential-steering-model.hpp"

DifferentialSteeringModel::DifferentialSteeringModel() noexcept:
        m_LeftWheelSpeedMutex{},
        m_RightWheelSpeedMutex{},
        m_LeftWheelSpeed{0.0f},
        m_RightWheelSpeed{0.0f}{
}

void DifferentialSteeringModel::setLeftWheelSpeed(opendlv::proxy::WheelSpeedRequest const &wheelSpeedRequest) noexcept {
    std::lock_guard<std::mutex> lock(m_LeftWheelSpeedMutex);
    m_LeftWheelSpeed = wheelSpeedRequest.wheelSpeed();
}

void DifferentialSteeringModel::setRightWheelSpeed(opendlv::proxy::WheelSpeedRequest const &wheelSpeedRequest) noexcept {
    std::lock_guard<std::mutex> lock(m_RightWheelSpeedMutex);
    m_RightWheelSpeed = wheelSpeedRequest.wheelSpeed();
}

opendlv::sim::KinematicState DifferentialSteeringModel::step(double) noexcept {

    float r{0.12};

    float leftWheelSpeedCopy;
    float rightWheelSpeedCopy;
    {
        std::lock_guard<std::mutex> lock1(m_LeftWheelSpeedMutex);
        std::lock_guard<std::mutex> lock2(m_RightWheelSpeedMutex);
        leftWheelSpeedCopy = m_LeftWheelSpeed;
        rightWheelSpeedCopy = m_RightWheelSpeed;
    }

    float yawRate = -(leftWheelSpeedCopy - rightWheelSpeedCopy) / (2 * r);
    float vx = (leftWheelSpeedCopy + rightWheelSpeedCopy) * 0.5f;

    opendlv::sim::KinematicState kinematicState;
    kinematicState.vx(vx);
    kinematicState.yawRate(yawRate);
    return kinematicState;
}
