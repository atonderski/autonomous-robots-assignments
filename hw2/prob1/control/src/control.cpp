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
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "behavior.hpp"


float getTimeSince(std::chrono::time_point<std::chrono::steady_clock> &previousTime) {
    auto currentTime = std::chrono::steady_clock::now();
    auto diff = currentTime - previousTime;
    return std::chrono::duration<float, std::milli>(diff).count();
}


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << " tests the Kiwi platform by sending actuation commands and reacting to sensor input."
                  << std::endl;
        std::cerr << "Usage:   " << argv[0]
                  << " --freq=<Integration frequency> --cid=<OpenDaVINCI session> [--verbose] [--demo]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
        retCode = 1;
    } else {
        bool const VERBOSE{commandlineArguments.count("verbose") != 0};
        uint16_t const CID = static_cast<const uint16_t>(std::stoi(commandlineArguments["cid"]));
        float const FREQ = std::stof(commandlineArguments["freq"]);

        double const DT = 1.0 / FREQ;
        double const WHEEL_RADIUS = 0.23;
        double const RADIUS = 0.5;

        cluon::OD4Session od4{CID};

        Behavior behavior(DT, 0.5, 0.2 * WHEEL_RADIUS, RADIUS);

        auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope) {
            auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            uint32_t const senderStamp = envelope.senderStamp();
            if (senderStamp == 0) {
                behavior.setFrontUltrasonic(distanceReading);
            } else {
                behavior.setRearUltrasonic(distanceReading);
            }
        }};
        auto onVoltageReading{[&behavior](cluon::data::Envelope &&envelope) {
            auto voltageReading = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
            uint32_t const senderStamp = envelope.senderStamp();
            if (senderStamp == 0) {
                behavior.setLeftIr(voltageReading);
            } else {
                behavior.setRightIr(voltageReading);
            }
        }};

        auto timeOfLastGPS = std::chrono::steady_clock::now();
        auto onFrame{[&behavior, &timeOfLastGPS](cluon::data::Envelope &&envelope) {
            auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
            // should put a lock on timeOfLastGPS but it's only POC here
            if (getTimeSince(timeOfLastGPS) >= 1000) {
                behavior.gpsMeasurment(frame.x(), frame.y());
                timeOfLastGPS = std::chrono::steady_clock::now();
            }
        }};

        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
        od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);
        od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);

        auto timeOfLastOdometer = std::chrono::steady_clock::now();
        auto atFrequency{[&VERBOSE, &behavior, &od4, &timeOfLastOdometer, &WHEEL_RADIUS]() -> bool {
            behavior.step();
            auto leftWheelSpeedRequest = behavior.getLeftWheelSpeed();
            auto rightWheelSpeedRequest = behavior.getRightWheelSpeed();

            if (getTimeSince(timeOfLastOdometer) >= 100) {
                double vL = static_cast<double>(leftWheelSpeedRequest.wheelSpeed());
                double vR = static_cast<double>(rightWheelSpeedRequest.wheelSpeed());
                behavior.wheelSpeedMeasurment(vL * WHEEL_RADIUS, vR * WHEEL_RADIUS);
                timeOfLastOdometer = std::chrono::steady_clock::now();
            }

            cluon::data::TimeStamp sampleTime;
            od4.send(leftWheelSpeedRequest, sampleTime, 0);
            // Sleep for a little bit since od4 can't seem to handle 2 similar messages at the same time...
            std::this_thread::sleep_for(std::chrono::duration<double>(0.005));
            od4.send(rightWheelSpeedRequest, sampleTime, 1);
            if (VERBOSE) {
                std::cout << "Ground steering angle is " << leftWheelSpeedRequest.wheelSpeed()
                          << " and pedal position is " << rightWheelSpeedRequest.wheelSpeed() << std::endl;
            }

            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);
    }
    return retCode;
}
