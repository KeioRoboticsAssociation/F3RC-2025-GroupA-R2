#include "Mechanism/OilArmController.hpp"
#include "variable.hpp"

namespace mechanism {

OilArmController::OilArmController(PseudoServo& arm_servo, PseudoServo& gripper_servo)
    : arm_servo_(arm_servo),
      gripper_servo_(gripper_servo),
      state_(State::kIdle) {}

void OilArmController::init() {
    setState(State::kInitialize);
}

void OilArmController::update() {
    switch (state_) {
        case State::kIdle:
            // Do nothing
            break;

        case State::kInitialize:
            arm_servo_.set_angle(oil_arm_config::kArmInitialAngle);
            gripper_servo_.set_angle(oil_arm_config::kGripperOpenAngle);
            if (!arm_servo_.is_moving() && !gripper_servo_.is_moving()) {
                setState(State::kIdle);
            }
            break;

        case State::kCollectStart:
            // Extend arm to approach the oil
            arm_servo_.set_angle(oil_arm_config::kArmExtendAngle);
            gripper_servo_.set_angle(oil_arm_config::kGripperOpenAngle);
            if (!arm_servo_.is_moving() && !gripper_servo_.is_moving()) {
                setState(State::kCollectGrip);
            }
            break;

        case State::kCollectGrip:
            // Grip the oil
            gripper_servo_.set_angle(oil_arm_config::kGripperCloseAngle);
            if (!gripper_servo_.is_moving()) {
                setState(State::kCollectLift);
            }
            break;

        case State::kCollectLift:
            // Lift the oil
            arm_servo_.set_angle(oil_arm_config::kArmLiftAngle);
            if (!arm_servo_.is_moving()) {
                setState(State::kIdle); // Collection sequence finished
            }
            break;

        case State::kPlaceStart:
            // Move arm to the placement position
            arm_servo_.set_angle(oil_arm_config::kArmPlaceAngle);
            if (!arm_servo_.is_moving()) {
                setState(State::kPlaceRelease);
            }
            break;

        case State::kPlaceRelease:
            // Release the oil
            gripper_servo_.set_angle(oil_arm_config::kGripperOpenAngle);
            if (!gripper_servo_.is_moving()) {
                setState(State::kPlaceRetract);
            }
            break;
        
        case State::kPlaceRetract:
            // Retract arm to the initial position
            arm_servo_.set_angle(oil_arm_config::kArmInitialAngle);
            if (!arm_servo_.is_moving()) {
                setState(State::kIdle); // Placement sequence finished
            }
            break;
    }
}

void OilArmController::collect() {
    if (!isBusy()) {
        setState(State::kCollectStart);
    }
}

void OilArmController::place() {
    if (!isBusy()) {
        setState(State::kPlaceStart);
    }
}

bool OilArmController::isBusy() const {
    return state_ != State::kIdle;
}

void OilArmController::stop() {
    // Stop motion by setting the target angle to the current angle
    arm_servo_.set_angle(arm_servo_.get_angle());
    gripper_servo_.set_angle(gripper_servo_.get_angle());
    setState(State::kIdle);
}

OilArmController::State OilArmController::getState() const {
    return state_;
}

void OilArmController::setState(State new_state) {
    state_ = new_state;
}

} // namespace mechanism