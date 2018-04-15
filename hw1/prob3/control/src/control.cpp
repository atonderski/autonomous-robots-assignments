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
        bool const DEMO{commandlineArguments.count("demo") != 0};

        cluon::OD4Session od4{CID};

        if (DEMO) {
            double const V0 = 0.5;
            double const T1 = 3;
            double const T2 = 10;
            double const DT = 1.0 / FREQ;
            double t = 0;
            auto demoAtFrequency{[&VERBOSE, &od4, &DT, &t, &V0, &T1, &T2]() -> bool {
                float leftWheelSpeed;
                float rightWheelSpeed;
                if (t < T1) {
                    leftWheelSpeed = 0.0f
                    rightWheelSpeed = static_cast<float>(V0 * t / T1);
                } else if (t < T2) {
                    leftWheelSpeed = static_cast<float>(V0 * (t - T1) / T2);
                    rightWheelSpeed = static_cast<float>(V0);
                }
                opendlv::proxy::WheelSpeedRequest leftWheelSpeedRequest;
                leftWheelSpeedRequest.wheelSpeed(leftWheelSpeed)
                opendlv::proxy::WheelSpeedRequest rightWheelSpeedRequest;
                rightWheelSpeedRequest.wheelSpeed(rightWheelSpeed);

                cluon::data::TimeStamp sampleTime;
                od4.send(leftWheelSpeedRequest, sampleTime, 0);
                od4.send(rightWheelSpeedRequest, sampleTime, 1);
                if (VERBOSE) {
                    std::cout << "Ground steering angle is " << leftWheelSpeedRequest.wheelSpeed()
                              << " and pedal position is " << rightWheelSpeedRequest.wheelSpeed() << std::endl;
                }
                t += DT;
                return true;
            }};
            od4.timeTrigger(FREQ, demoAtFrequency);
        } else {
            Behavior behavior;

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

            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
            od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);

            auto atFrequency{[&VERBOSE, &behavior, &od4]() -> bool {
                behavior.step();
                auto leftWheelSpeedRequest = behavior.getLeftWheelSpeed();
                auto rightWheelSpeedRequest = behavior.getRightWheelSpeed();

                cluon::data::TimeStamp sampleTime;
                od4.send(leftWheelSpeedRequest, sampleTime, 0);
                od4.send(rightWheelSpeedRequest, sampleTime, 1);
                if (VERBOSE) {
                    std::cout << "Ground steering angle is " << leftWheelSpeedRequest.wheelSpeed()
                              << " and pedal position is " << rightWheelSpeedRequest.wheelSpeed() << std::endl;
                }

                return true;
            }};

            od4.timeTrigger(FREQ, atFrequency);
        }
    }
    return retCode;
}
