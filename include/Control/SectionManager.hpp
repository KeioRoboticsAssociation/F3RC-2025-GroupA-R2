#include <vector>
#include <memory>
#include "Control/StateEstimator.hpp"
#include "Control/BaseSection.hpp"

class SectionManager {
public:
    SectionManager(std::vector<std::unique_ptr<BaseSection>>&& sections)
        : sections(std::move(sections)), current_index(0) {}

    void update(StateEstimator& state, BehaviorController& behavior, double dt) {
        if (current_index >= sections.size()) return;

        auto& current = sections[current_index];
        current->run(state, behavior, dt);

        if (current->isFinished()) {
            current_index++;
        }
    }

    bool allFinished() const {
        return current_index >= sections.size();
    }

private:
    std::vector<std::unique_ptr<BaseSection>> sections;
    size_t current_index;
};
