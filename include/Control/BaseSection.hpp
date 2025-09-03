#pragma once
#include "Control/StateEstimator.hpp"
#include "Control/BehaviorController.hpp"

class BaseSection {
public:
    virtual ~BaseSection() {}
    // 1周期分の処理
    virtual void run(StateEstimator& state, BehaviorController& behavior, double dt) = 0;
    // このセクションが終わったかどうか
    virtual bool isFinished() const = 0;
};
