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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "differential-steering-model.hpp"

TEST_CASE("Test differential steering model, zero speed should always give zero output.") {
    DifferentialSteeringModel dsm;

    opendlv::proxy::WheelSpeedRequest leftRequest;
    leftRequest.wheelSpeed(0.f);
    dsm.setLeftWheelSpeed(leftRequest);

    opendlv::proxy::WheelSpeedRequest rightRequest;
    rightRequest.wheelSpeed(0.f);
    dsm.setRightWheelSpeed(rightRequest);

    opendlv::sim::KinematicState ks = dsm.step(0.01);
    REQUIRE(ks.vx() == Approx(0.0f));
    REQUIRE(ks.vy() == Approx(0.0f));
    REQUIRE(ks.vz() == Approx(0.0f));
    REQUIRE(ks.rollRate() == Approx(0.0f));
    REQUIRE(ks.pitchRate() == Approx(0.0f));
    REQUIRE(ks.yawRate() == Approx(0.0f));
}

TEST_CASE("Test differential steering model, going straight ahead should give positive vx behaviour") {
    DifferentialSteeringModel dsm;

    opendlv::proxy::WheelSpeedRequest leftRequest;
    leftRequest.wheelSpeed(0.1f);
    dsm.setLeftWheelSpeed(leftRequest);

    opendlv::proxy::WheelSpeedRequest rightRequest;
    rightRequest.wheelSpeed(0.1f);
    dsm.setRightWheelSpeed(rightRequest);

    float prevLongitudinalSpeed{0.0f};
    float longitudinalSpeed{0.0f};
    for (uint16_t i{0}; i < 100; i++) {
        opendlv::sim::KinematicState ks = dsm.step(0.01);
        prevLongitudinalSpeed = longitudinalSpeed;
        longitudinalSpeed = ks.vx();
        REQUIRE(longitudinalSpeed >= 0.0f);
        REQUIRE(1+ks.vy() == Approx(1.0f));
        REQUIRE(ks.vz() == Approx(0.0f));
        REQUIRE(ks.rollRate() == Approx(0.0f));
        REQUIRE(ks.pitchRate() == Approx(0.0f));
        REQUIRE(ks.yawRate() == Approx(0.0f));
    }
    std::cout << longitudinalSpeed << " " << prevLongitudinalSpeed << std::endl;
    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
}

TEST_CASE("Test differential steering model, going in reverse should give negative longitudinal constat behaviour.") {
    DifferentialSteeringModel dsm;

    opendlv::proxy::WheelSpeedRequest leftRequest;
    leftRequest.wheelSpeed(-0.1f);
    dsm.setLeftWheelSpeed(leftRequest);

    opendlv::proxy::WheelSpeedRequest rightRequest;
    rightRequest.wheelSpeed(-0.1f);
    dsm.setRightWheelSpeed(rightRequest);

    float prevLongitudinalSpeed{0.0f};
    float longitudinalSpeed{0.0f};
    for (uint16_t i{0}; i < 100; i++) {
        opendlv::sim::KinematicState ks = dsm.step(0.01);
        prevLongitudinalSpeed = longitudinalSpeed;
        longitudinalSpeed = ks.vx();
        REQUIRE(longitudinalSpeed <= 0.0f);
        REQUIRE(1+ks.vy() == Approx(1.0f)); //FOR SOME REASON DOESN'T WORK!! it complains that -0.0f isnt approx 0.0f...
        REQUIRE(ks.vz() == Approx(0.0f));
        REQUIRE(ks.rollRate() == Approx(0.0f));
        REQUIRE(ks.pitchRate() == Approx(0.0f));
        REQUIRE(ks.yawRate() == Approx(0.0f));
    }
    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
}

TEST_CASE("Test differential steering model, right speed higher than left should steer to the left.") {
    DifferentialSteeringModel dsm;

    opendlv::proxy::WheelSpeedRequest leftRequest;
    leftRequest.wheelSpeed(0.1f);
    dsm.setLeftWheelSpeed(leftRequest);

    opendlv::proxy::WheelSpeedRequest rightRequest;
    rightRequest.wheelSpeed(0.2f);
    dsm.setRightWheelSpeed(rightRequest);

    float prevLateralSpeed{0.0f};
    float prevLongitudinalSpeed{0.0f};
    float prevYawRate{0.0f};
    float lateralSpeed{0.0f};
    float longitudinalSpeed{0.0f};
    float yawRate{0.0f};
    for (uint16_t i{0}; i < 100; i++) {
        opendlv::sim::KinematicState ks = dsm.step(0.01);
        prevLateralSpeed = lateralSpeed;
        prevLongitudinalSpeed = longitudinalSpeed;
        prevYawRate = yawRate;
        lateralSpeed = ks.vy();
        longitudinalSpeed = ks.vx();
        yawRate = ks.yawRate();
        REQUIRE(lateralSpeed >= 0.0f);
        REQUIRE(yawRate >= 0.0f);
    }
    REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
    REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
}

//TEST_CASE(
//        "Test differential steering model, steering to the left while reversing should give movement to the left and CW rotation, and find steady state.") {
//    DifferentialSteeringModel dsm;
//
//    opendlv::proxy::GroundSteeringRequest gsr;
//    gsr.groundSteering(0.3f);
//    dsm.setGroundSteeringAngle(gsr);
//
//    opendlv::proxy::PedalPositionRequest ppr;
//    ppr.position(-0.2f);
//    dsm.setPedalPosition(ppr);
//
//    float prevLateralSpeed{0.0f};
//    float prevLongitudinalSpeed{0.0f};
//    float prevYawRate{0.0f};
//    float lateralSpeed{0.0f};
//    float longitudinalSpeed{0.0f};
//    float yawRate{0.0f};
//    for (uint16_t i{0}; i < 100; i++) {
//        opendlv::sim::KinematicState ks = dsm.step(0.01);
//        prevLateralSpeed = lateralSpeed;
//        prevLongitudinalSpeed = longitudinalSpeed;
//        prevYawRate = yawRate;
//        lateralSpeed = ks.vy();
//        longitudinalSpeed = ks.vx();
//        yawRate = ks.yawRate();
//        REQUIRE(lateralSpeed >= 0.0f);
//        REQUIRE(yawRate >= 0.0f);
//    }
//    REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
//    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
//    REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
//}
//
//TEST_CASE(
//        "Test differential steering model, steering to the right should give movement to the right and CW rotation, and find steady state.") {
//    DifferentialSteeringModel dsm;
//
//    opendlv::proxy::GroundSteeringRequest gsr;
//    gsr.groundSteering(-0.3f);
//    dsm.setGroundSteeringAngle(gsr);
//
//    opendlv::proxy::PedalPositionRequest ppr;
//    ppr.position(0.2f);
//    dsm.setPedalPosition(ppr);
//
//    float prevLateralSpeed{0.0f};
//    float prevLongitudinalSpeed{0.0f};
//    float prevYawRate{0.0f};
//    float lateralSpeed{0.0f};
//    float longitudinalSpeed{0.0f};
//    float yawRate{0.0f};
//    for (uint16_t i{0}; i < 100; i++) {
//        opendlv::sim::KinematicState ks = dsm.step(0.01);
//        prevLateralSpeed = lateralSpeed;
//        prevLongitudinalSpeed = longitudinalSpeed;
//        prevYawRate = yawRate;
//        lateralSpeed = ks.vy();
//        longitudinalSpeed = ks.vx();
//        yawRate = ks.yawRate();
//        REQUIRE(lateralSpeed <= 0.0f);
//        REQUIRE(yawRate <= 0.0f);
//    }
//    REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
//    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
//    REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
//}
//
//TEST_CASE(
//        "Test differential steering model, steering to the right while reversing should give movement to the right and CCW rotation, and find steady state.") {
//    DifferentialSteeringModel dsm;
//
//    opendlv::proxy::GroundSteeringRequest gsr;
//    gsr.groundSteering(-0.3f);
//    dsm.setGroundSteeringAngle(gsr);
//
//    opendlv::proxy::PedalPositionRequest ppr;
//    ppr.position(-0.2f);
//    dsm.setPedalPosition(ppr);
//
//    float prevLateralSpeed{0.0f};
//    float prevLongitudinalSpeed{0.0f};
//    float prevYawRate{0.0f};
//    float lateralSpeed{0.0f};
//    float longitudinalSpeed{0.0f};
//    float yawRate{0.0f};
//    for (uint16_t i{0}; i < 100; i++) {
//        opendlv::sim::KinematicState ks = dsm.step(0.01);
//        prevLateralSpeed = lateralSpeed;
//        prevLongitudinalSpeed = longitudinalSpeed;
//        prevYawRate = yawRate;
//        lateralSpeed = ks.vy();
//        longitudinalSpeed = ks.vx();
//        yawRate = ks.yawRate();
//        REQUIRE(lateralSpeed <= 0.0f);
//        REQUIRE(yawRate <= 0.0f);
//    }
//    REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
//    REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
//    REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
//}
//
//TEST_CASE("Test differential steering model, steer to the left with slowly increasing amplitude.") {
//    DifferentialSteeringModel dsm;
//
//    opendlv::proxy::GroundSteeringRequest gsr;
//    gsr.groundSteering(0.0f);
//    dsm.setGroundSteeringAngle(gsr);
//
//    opendlv::proxy::PedalPositionRequest ppr;
//    ppr.position(0.1f);
//    dsm.setPedalPosition(ppr);
//
//    for (uint16_t i{0}; i < 100; i++) {
//        opendlv::sim::KinematicState ks = dsm.step(0.01);
//        float lateralSpeed = ks.vy();
//        float yawRate = ks.yawRate();
//        REQUIRE(lateralSpeed >= 0.0f);
//        REQUIRE(lateralSpeed < 10.0f);
//        REQUIRE(yawRate >= 0.0f);
//        REQUIRE(yawRate < 10.0f);
//
//        if (gsr.groundSteering() < 0.5f) {
//            gsr.groundSteering(gsr.groundSteering() + 0.01f);
//            dsm.setGroundSteeringAngle(gsr);
//        }
//    }
//}
//
//TEST_CASE("Test differential steering model, steer to the left with slowly increasing speed.") {
//    DifferentialSteeringModel dsm;
//
//    opendlv::proxy::GroundSteeringRequest gsr;
//    gsr.groundSteering(0.2f);
//    dsm.setGroundSteeringAngle(gsr);
//
//    opendlv::proxy::PedalPositionRequest ppr;
//    ppr.position(0.0f);
//    dsm.setPedalPosition(ppr);
//
//    for (uint16_t i{0}; i < 100; i++) {
//        opendlv::sim::KinematicState ks = dsm.step(0.01);
//        float lateralSpeed = ks.vy();
//        float yawRate = ks.yawRate();
//
//        REQUIRE(lateralSpeed >= 0.0f);
//        REQUIRE(lateralSpeed < 10.0f);
//        REQUIRE(yawRate >= 0.0f);
//        REQUIRE(yawRate < 10.0f);
//
//        ppr.position(ppr.position() + 0.01f);
//        dsm.setPedalPosition(ppr);
//    }
//}
