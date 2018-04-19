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
#include "differential-steering-model.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << " is a dynamics model for the Chalmers Kiwi platform." << std::endl;
        std::cerr << "Usage:   " << argv[0]
                  << " --freq=<Model frequency> --cid=<OpenDaVINCI session>"
                  << std::endl;
        std::cerr << "Example: " << argv[0] << " --freq=100 --cid=111" << std::endl;
        return 1;
    }

    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = static_cast<const uint16_t>(std::stoi(commandlineArguments["cid"]));
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;

    DifferentialSteeringModel differentialSteeringModel;

    auto onWheelSpeedRequest{[&differentialSteeringModel](cluon::data::Envelope &&envelope) {
        auto wheelSpeedRequest = cluon::extractMessage<opendlv::proxy::WheelSpeedRequest>(std::move(envelope));
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
            differentialSteeringModel.setLeftWheelSpeed(wheelSpeedRequest);
        } else if (senderStamp == 1) {
            differentialSteeringModel.setRightWheelSpeed(wheelSpeedRequest);
        }
    }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::WheelSpeedRequest::ID(), onWheelSpeedRequest);

    auto atFrequency{[&VERBOSE, &DT, &differentialSteeringModel, &od4]() -> bool {
        opendlv::sim::KinematicState kinematicState = differentialSteeringModel.step(DT);
        cluon::data::TimeStamp sampleTime;
        od4.send(kinematicState, sampleTime);
        if (VERBOSE) {
            std::cout << "Kinematic state"
                      << " is at velocity [vx=" << kinematicState.vx() << ", vy=" << kinematicState.vy() << ", vz="
                      << kinematicState.vz() << "] with the rotation rate [rollRate=" << kinematicState.rollRate()
                      << ", pitchRate="
                      << kinematicState.pitchRate() << ", yawRate=" << kinematicState.yawRate() << "]."
                      << std::endl;
        }
        return true;
    }};

    od4.timeTrigger(FREQ, atFrequency);

    return retCode;
}
