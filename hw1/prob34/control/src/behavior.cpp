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

#include <iostream>
#include <cmath>
#include "behavior.hpp"

Behavior::Behavior() noexcept:
        m_frontUltrasonicReading{},
        m_rearUltrasonicReading{},
        m_leftIrReading{},
        m_rightIrReading{},
        m_leftWheelSpeedRequest{},
        m_rightWheelSpeedRequest{},
        m_frontUltrasonicReadingMutex{},
        m_rearUltrasonicReadingMutex{},
        m_leftIrReadingMutex{},
        m_rightIrReadingMutex{},
        m_leftWheelSpeedRequestMutex{},
        m_rightWheelSpeedRequestMutex{},
        m_preferedDirection{0.5f} {
}

opendlv::proxy::WheelSpeedRequest Behavior::getLeftWheelSpeed() noexcept {
    std::lock_guard<std::mutex> lock(m_leftWheelSpeedRequestMutex);
    return m_leftWheelSpeedRequest;
}

opendlv::proxy::WheelSpeedRequest Behavior::getRightWheelSpeed() noexcept {
    std::lock_guard<std::mutex> lock(m_rightWheelSpeedRequestMutex);
    return m_rightWheelSpeedRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept {
    std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
    m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept {
    std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
    m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept {
    std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
    m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept {
    std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
    m_rightIrReading = rightIrReading;
}


void Behavior::step() noexcept {
    float frontDistance;
    float rearDistance;
    double leftDistance;
    double rightDistance;
    {
        std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
        std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
        std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
        std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);

        frontDistance = m_frontUltrasonicReading.distance();
        rearDistance = m_rearUltrasonicReading.distance();
        leftDistance = convertIrVoltageToDistance(m_leftIrReading.voltage());
        rightDistance = convertIrVoltageToDistance(m_rightIrReading.voltage());
    }

    float avoidanceSpeedFactor{0.0f};
    // Change speed based on proximity in front/back
    if (frontDistance < 1.f) {
        avoidanceSpeedFactor -= 0.6f * (1.f - frontDistance);
    }
    if (rearDistance < 0.5f) {
        avoidanceSpeedFactor += 0.6f * (0.5f - rearDistance) / 0.5f;
    }
    // add the default speed
    float speed = avoidanceSpeedFactor + 0.4f;

    float avoidanceTurningFactor{0.f};

    // Avoid obstacles in front and back
    if (frontDistance < 0.5f || rearDistance < 0.5f) {
        if (leftDistance < rightDistance) {
            m_preferedDirection = -fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        } else {
            m_preferedDirection = fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        }
        avoidanceTurningFactor = m_preferedDirection;
    }

    bool oldIsFollowingWall = m_isFollowingWall;
    // Avoid obstacles on the sides
    double sideAvoidanceThreshold{0.25f};
    double sideDetectionThreshold{0.32f};
    m_isFollowingWall = false;
    if (leftDistance < sideDetectionThreshold) {
        std::cout << "Close on the left: " << leftDistance << std::endl;
        m_isFollowingWall = true;
        if (leftDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor -= 0.4 * (1 - leftDistance / sideAvoidanceThreshold);
        }
    }
    if (rightDistance < sideDetectionThreshold) {
        std::cout << "Close on the right: " << rightDistance << std::endl;
        m_isFollowingWall = true;
        if (rightDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor += 0.4f * (1 - rightDistance / sideAvoidanceThreshold);
        }
    }

    // if we aren't close to a wall, we have a chance to change direction
    if (!m_isFollowingWall) {
        if (oldIsFollowingWall || random() % 50 == 0) {
            m_preferedDirection = -m_preferedDirection;
            std::cout << "Changed direction to " << m_preferedDirection << std::endl;
        }
    }
    float turningAngle = avoidanceTurningFactor == 0.0f ? m_preferedDirection : avoidanceTurningFactor;

    float r{0.12};
    float leftWheelSpeed = speed - turningAngle * r;
    float rightWheelSpeed = speed + turningAngle * r;

    {
        std::lock_guard<std::mutex> lock1(m_leftWheelSpeedRequestMutex);
        std::lock_guard<std::mutex> lock2(m_rightWheelSpeedRequestMutex);

        opendlv::proxy::WheelSpeedRequest leftWheelSpeedRequest;
        leftWheelSpeedRequest.wheelSpeed(leftWheelSpeed);
        m_leftWheelSpeedRequest = leftWheelSpeedRequest;

        opendlv::proxy::WheelSpeedRequest rightWheelSpeedRequest;
        rightWheelSpeedRequest.wheelSpeed(rightWheelSpeed);
        m_rightWheelSpeedRequest = rightWheelSpeedRequest;
    }
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept {
    double voltageDividerR1 = 1000.0;
    double voltageDividerR2 = 1000.0;

    double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
    double distance = (3.4 - sensorVoltage) / 9.9;
    return distance;
}
