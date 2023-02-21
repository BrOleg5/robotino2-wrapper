#include "robotino2.hpp"

#include "rec/core_lt/Timer.h"
#include "rec/core_lt/utils.h"

int main( int argc, char **argv )
{
    //Default IP addres Robotino 2
    std::string ip_addr = "172.26.1.0";
    if (argc > 1)
    {
        ip_addr = argv[1];
    }
    Robotino2 robotino(ip_addr);
    rec::core_lt::Timer timer;
    
    unsigned int test_duration = 10000;

    std::cout << "| Time | M1 pos | M2 pos | M3 pos | M1 vel | M2 vel | M3 vel| M1 cur | M2 cur | M3 cur |" << std::endl;
    std::cout << "|--------------------------------------------------------------------------------------|" << std::endl;
    timer.start();
    while (timer.msecsElapsed() <= test_duration)
    {
        try {
            robotino.set_robot_speed((float)0.1, (float)0, (float)0);
        }
        catch(const std::invalid_argument& e) {
            std::cout << "set_robot_speed() failed with error: " << e.what();
        }
        std::cout << " | " << timer.msecsElapsed() << " | " << robotino.get_actual_position(0) << " | " << robotino.get_actual_position(1) << " | ";
        std::cout << robotino.get_actual_position(2) << " | " << robotino.get_actual_velocity(0) << " | " << robotino.get_actual_velocity(1) << " | ";
        std::cout << robotino.get_actual_velocity(2) << " | " << robotino.get_actual_current(0) << " | " << robotino.get_actual_current(1) << " | ";
        std::cout << robotino.get_actual_current(2) << " | " << std::endl;
        rec::core_lt::msleep(20);
    }
    return 0;
}
