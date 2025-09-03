#include "Control/BaseSection.hpp"

class Section1 : public BaseSection {
public:
    void run(StateEstimator& state, BehaviorController& behavior, double dt) override {
        Pose2D pose = state.getRelativePosition();
        
        // x < 1.0 までは前進
        if (pose.x < 1.0) {
            behavior.setTargetVelocity(0.3, 0.0);
        } else {
            behavior.stop();
            finished = true;
        }
    }

    bool isFinished() const override { return finished; }

private:
    bool finished = false;
};
