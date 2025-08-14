# include <iostream>
# include "Driver/LimitSwitch.hpp"
# include "config.hpp"

int main(){
    std::cout << std::boolalpha; 
    LimitSwitch ls(PinsForSensor::FRONT_LIMIT);

    ls.init();
    std::cout << "Now Initializing..." << std::endl;
    
    while(true){
        // std::cout << "LimitSwitch.isPressed():" << ls.isPressed() << std::endl;
        // std::cout << "LimitSwitch.isPressedEdge():" << ls.isPressedEdge() << std::endl;
        // std::cout << "LimitSwitch.getRawInput():" << ls.getRawInput() << std::endl;
    }
    return 0;
}