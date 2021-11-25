#ifndef ROBOTINO2_HPP
#   define ROBOTINO2_HPP 

#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "rec/robotino/com/all.h"
#include "rec/core_lt/Timer.h"
#include "rec/core_lt/utils.h"
using namespace rec::robotino::com;

#ifndef PI
#   define PI 3.14159265358979323846
#endif

/**
 * @class MyCom
 * @brief Implementation of Com() class.
 */
class MyCom : public Com
{
private:

    /**
     * Number of Robotino motors.
     */
    const size_t motor_num = 3;

    /**
     * Array of Robotino motors.
     */
    Motor motor[3];

    /**
     * Motors' actual positions in encoder ticks. 
     */
    std::vector<int> actual_position = {0, 0, 0};
    
    /**
     * Motors' actual velocities in rad/s. 
     */
    std::vector<float> actual_velocity = {0.0, 0.0, 0.0};

    /**
     * Motors' actual currents in A. 
     */
    std::vector<float> actual_current = {0.0, 0.0, 0.0};

public:

	MyCom() : Com() {}

    /**
     * This function is called on errors.
     * 
     * @param error the error which caused this event.
     * @param errorString a human readable error description.
     * @throws noting
     * @remark This function is called from outside the applications main thread.
     * This is extremely important particularly with regard to GUI applications.
     */
	void errorEvent(Error error, const char* errorString);

    /**
     * This function is called if a connection to Robotino has been established.
     * 
     * @throws nothing
     * @remark This function is called from outside the applications main thread. 
     * This is extremely important particularly with regard to GUI applications.
     */
	void connectedEvent();

    /**
     * This function is called when a connection is closed.
     * 
     * @throws nothing
     * @remark This function is called from outside the applications main thread.
     * This is extremely important particularly with regard to GUI applications.
     */
	void connectionClosedEvent();

    /**
     * This function is called whenever sensor readings have been updated.
     * 
     * @throws nothing
     */
    void updateEvent();

    /**
     * Encoder position of motor.
     * 
     * @param num motor number.
     * @return actual position of this motor in encoder ticks.
     */
    int get_actual_position(size_t num);

    /**
     * Encoder position of all motors.
     * 
     * @return actual positionss of all motors in encoder ticks.
     */
    std::vector<int> get_actual_positions();

    /**
     * @param num motor number.
     * @return actual velocity of this motor in rad/s.
     */
    float get_actual_velocity(size_t num);

    /**
     * @return actual velocities of all motors in rad/s.
     */
    std::vector<float> get_actual_velocities();

    /**
     * @param num motor number.
     * @return current of this motor in A.
     */
    float get_actual_current(size_t num);

    /**
     * @return currents of all motor in A.
     */
    std::vector<float> get_actual_currents();
};

/**
 * @class Robotino2
 * @brief OpenRobotinoAPI wrapper.
 */
class Robotino2
{
private:

    /**
     * Number of Robotino motors.
     */
    const size_t motor_num = 3;

    /**
     * Robotino comunication interface.
     */
    MyCom com;

    /**
     * Represents a Robotino motion drive.
     */
    OmniDrive omniDrive;

    /**
     * Array of Robotino motors.
     */
    Motor motor[3];

    /**
     * Timer.
     */
    rec::core_lt::Timer timer;

    /**
     * Motor's velocity limit in rad/s.
     */
    const float motor_vel_limit = 240;

    /**
     * Robot's linear speed limit in m/s.
     */
    const float robot_lin_speed_limit = 0.8f;

    /**
     * Robot's angular velocity limit in rad/s.
     */
    const float robot_vel_limit = (float) PI;

public:

    /**
     * Constructor. Start connection with Robotino4.
     * 
     * @param ip_addr IP-addres of Robotino4.
     * @param sample_time sample time of motors' control and sensors' measurements in ms. Default 20 ms.
     */
    Robotino2(const std::string& ip_addr, unsigned int sample_time=20);

    /**
     * Destructor. Close connection with Robotino.
     */
    ~Robotino2();

    /**
     * Encoder position of motor.
     * 
     * @param num motor number.
     * @return actual position of this motor in encoder ticks.
     */
    int get_actual_position(size_t num);

    /**
     * Encoder position of all motors.
     * 
     * @return actual positionss of all motors in encoder ticks.
     */
    std::vector<int> get_actual_positions();

    /**
     * @param num motor number.
     * @return actual velocity of this motor in rad/s.
     */
    float get_actual_velocity(size_t num);

    /**
     * @return actual velocities of all motors in rad/s.
     */
    std::vector<float> get_actual_velocities();

    /**
     * @param num motor number.
     * @return current of this motor in A.
     */
    float get_actual_current(size_t num);

    /**
     * @return currents of all motor in A.
     */
    std::vector<float> get_actual_currents();

    /**
     * Sets the setpoint speed of this motor.
     * 
     * @param num motor number.
     * @param speed setpoint speed in rad/s.
     * @throw Invalid argument.
     */
    void set_motor_speed(size_t num, float speed);

    /**
     * Sets the setpoint speed of all motors.
     * 
     * @param speed speed setpoints for all motors in rad/s.
     * @throw Invalid argument.
     */
    void set_motors_speed(const std::vector<float>& speeds);

    /**
     * Resets the position of this motor.
     * 
     * @param num motor number.
     */
    void reset_motor_position(size_t num);

    /**
     * Resets the position of all motors.
     */
    void reset_motors_position();

    /**
     * Sets the proportional, integral and differential constants of the PID controller.
     * 
     * @param num motor number.
     * @param kp proportional constant of the motor's speed PID controller.
     * @param ki integral constant of the motor's speed PID controller.
     * @param kd differential constant of the motor's speed PID controller.
     * @remark These values are scaled by the microcontroller firmware to match with the PID controller implementation.
     * If value is given, the microcontroller firmware uses its build in default value.
     * Robotino v3 Parameters are floating point values used by the microcontroller directly. If parameter is less than 0 the default parameter is used.
     */
    void set_motor_pid(size_t num, unsigned char kp, unsigned char ki, unsigned char kd);

    /**
     * Sets the proportional, integral and differential constants of the PID controllers.
     * 
     * @param kp proportional constants of the motor' speed PID controllers.
     * @param ki integral constants of the motor' speed PID controllers.
     * @param kd differential constants of the motor' speed PID controllers.
     * @remark These values are scaled by the microcontroller firmware to match with the PID controller implementation.
     * If value is given, the microcontroller firmware uses its build in default value.
     * Robotino v3 Parameters are floating point values used by the microcontroller directly. If parameter is less than 0 the default parameter is used.
     */
    void set_motors_pid(const std::vector<unsigned char>& kp, const std::vector<unsigned char>& ki, const std::vector<unsigned char>& kd);

    /**
     * Set robot speed.
     * 
     * @param vx speed along x axis of robot's local coordinate system in m/s.
     * @param vy speed along y axis of robot's local coordinate system in m/s.
     * @param omega angular velocity of rotation in rad/s.
     * @throw Invalid argument.
     * @remark This function is thread save. It should be called about every 100ms.
     */
    void set_robot_speed(float vx, float vy, float omega);

    /**
     * Project the velocity of the robot in cartesian coordinates to single motor speeds.
     * 
     * @param vx speed along x axis of robot's local coordinate system in m/s.
     * @param vy speed along y axis of robot's local coordinate system in m/s.
     * @param omega angular velocity of rotation in rad/s.
     */
    std::vector<float> robot_speed_to_motor_speeds(float vx, float vy, float omega);

};

#endif