#ifndef COAL_ARM_CONTROLLER_HPP
#define COAL_ARM_CONTROLLER_HPP

#include "Parts/ServoMotor.hpp"
#include "Mechanism/Arms/PseudoServo.hpp"

class CoalArmController {
public:
    CoalArmController(PseudoServo& armServo, Servo& gripServo);

    void init();
    void collect();
    void place();

    bool isBusy() const;

    void stop();
    void update();

private:
    enum class State {
        Idle,
        Init,
        Collect_ArmDown,
        Collect_GripClose,
        Collect_ArmUp,
        Place_ArmDown,
        Place_GripOpen,
        Place_ArmUp,
        EmergencyStop
    };
    enum class Command {
        None,
        Collect,
        Place
    };

    PseudoServo& _arm;
    Servo& _grip;

    State _state;
    Command _pending;
    Timer _state_timer;
};

#endif
