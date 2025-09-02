#include "Mechanism/CoalArmController.hpp"
#include "config.hpp"

CoalArmController::CoalArmController(PseudoServo& armServo, Servo& gripServo)
    : _arm(armServo), _grip(gripServo), _state(State::Idle) {}

void CoalArmController::init() {
    _state = State::Init;
    _state_timer.reset();
    _state_timer.start();
}

void CoalArmController::collect() {
    if (isBusy()) {
        _pending = Command::Collect;
    } else {
        _state = State::Collect_ArmDown;
        _state_timer.reset();
        _state_timer.start();
        _pending = Command::None;
    }
}

void CoalArmController::place() {
    if (isBusy()) {
        _pending = Command::Place;
    } else {
        _state = State::Place_ArmDown;
        _state_timer.reset();
        _state_timer.start();
        _pending = Command::None;
    }
}

bool CoalArmController::isBusy() const {
    return _state != State::Idle;
}

void CoalArmController::stop() {
    _arm.home();
    _grip.setAngleDeg(CoalArmParameter::GRIP_OPEN_ANGLE);
    _state = State::EmergencyStop;
    _state_timer.stop();
}

/*! 外部の Ticker から呼び出すこと！！！ */
void CoalArmController::update() {
    switch (_state) {
        case State::Init:
            _arm.init();
            _grip.setAngleDeg(CoalArmParameter::GRIP_OPEN_ANGLE);
            if (_arm.isHomed()) {
                _state = State::Idle;
                _state_timer.stop();
            }
            break;
        case State::Collect_ArmDown:
            _arm.setAngleDeg(CoalArmParameter::ARM_DOWN_ANGLE);
            if (_arm.getAngleDeg() <= CoalArmParameter::ARM_DOWN_THRESHOLD || _state_timer.read_ms() > CoalArmParameter::ARM_MOVE_TIMEOUT_MS) {
                _state = State::Collect_GripClose;
                _state_timer.reset();
            }
            break;
        case State::Collect_GripClose:
            _grip.setAngleDeg(CoalArmParameter::GRIP_CLOSE_ANGLE);
            if (_state_timer.read_ms() > CoalArmParameter::GRIP_ACTION_TIMEOUT_MS) {
                _state = State::Collect_ArmUp;
                _state_timer.reset();
            }
            break;
        case State::Collect_ArmUp:
            _arm.setAngleDeg(CoalArmParameter::ARM_UP_ANGLE);
            if (_arm.getAngleDeg() >= CoalArmParameter::ARM_UP_THRESHOLD || _state_timer.read_ms() > CoalArmParameter::ARM_MOVE_TIMEOUT_MS) {
                _state = State::Idle;
                _state_timer.stop();
            }
            break;
        case State::Place_ArmDown:
            _arm.setAngleDeg(CoalArmParameter::ARM_DOWN_ANGLE);
            if (_arm.getAngleDeg() <= CoalArmParameter::ARM_DOWN_THRESHOLD || _state_timer.read_ms() > CoalArmParameter::ARM_MOVE_TIMEOUT_MS) {
                _state = State::Place_GripOpen;
                _state_timer.reset();
            }
            break;
        case State::Place_GripOpen:
            _grip.setAngleDeg(CoalArmParameter::GRIP_OPEN_ANGLE);
            if (_state_timer.read_ms() > CoalArmParameter::GRIP_ACTION_TIMEOUT_MS) {
                _state = State::Place_ArmUp;
                _state_timer.reset();
            }
            break;
        case State::Place_ArmUp:
            _arm.setAngleDeg(CoalArmParameter::ARM_UP_ANGLE);
            if (_arm.getAngleDeg() >= CoalArmParameter::ARM_UP_THRESHOLD || _state_timer.read_ms() > CoalArmParameter::ARM_MOVE_TIMEOUT_MS) {
                _state = State::Idle;
                _state_timer.stop();
            }
            break;
        case State::EmergencyStop:
            break;
        case State::Idle:
            if (_pending == Command::Collect) {
                collect();
            } else if (_pending == Command::Place) {
                place();
            }
            break;
    }
}
