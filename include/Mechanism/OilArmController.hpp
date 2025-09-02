#ifndef INCLUDED_MECHANISM_OIL_ARM_CONTROLLER_HPP_
#define INCLUDED_MECHANISM_OIL_ARM_CONTROLLER_HPP_

#include "Mechanism/PseudoServo.hpp"

namespace mechanism {

class OilArmController {
public:
    /**
     * @brief State of the oil arm
     */
    enum class State {
        kIdle,
        kInitialize,
        kCollectStart,
        kCollectGrip,
        kCollectLift,
        kPlaceStart,
        kPlaceRelease,
        kPlaceRetract,
    };

    /**
     * @brief Construct a new Oil Arm Controller object
     * 
     * @param arm_servo Reference to the pseudo servo for the arm
     * @param gripper_servo Reference to the pseudo servo for the gripper
     */
    OilArmController(PseudoServo& arm_servo, PseudoServo& gripper_servo);

    /**
     * @brief Initialize the arm to the initial position
     */
    void init();

    /**
     * @brief Update the state machine of the arm. This should be called periodically.
     */
    void update();

    /**
     * @brief Start the oil collection sequence (non-blocking)
     */
    void collect();

    /**
     * @brief Start the oil placement sequence (non-blocking)
     */
    void place();

    /**
     * @brief Check if the arm is busy with a sequence
     * 
     * @return true if the arm is busy, false otherwise
     */
    bool isBusy() const;

    /**
     * @brief Stop the current motion immediately
     */
    void stop();

    /**
     * @brief Get the current state of the arm (for testing/debugging)
     * 
     * @return State The current state
     */
    State getState() const;

private:
    PseudoServo& arm_servo_;
    PseudoServo& gripper_servo_;

    State state_;
    
    /**
     * @brief Set the new state for the state machine
     * 
     * @param new_state The new state
     */
    void setState(State new_state);
};

}  // namespace mechanism

#endif  // INCLUDED_MECHANISM_OIL_ARM_CONTROLLER_HPP_
