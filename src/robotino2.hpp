#ifndef ROBOTINO2_HPP
#define ROBOTINO2_HPP 

#include <iostream>
#include <cmath>
#include <array>
#include <stdexcept>
#include "rec/robotino/com/Com.h"
#include "rec/robotino/com/ComId.h"
#include "rec/robotino/com/Motor.h"
#include "rec/robotino/com/OmniDrive.h"
using namespace rec::robotino::com;

#ifndef PI
#   define PI 3.14159265358979323846f
#endif

/**
 * @class Robotino2
 * @brief OpenRobotinoAPI wrapper.
 */
class Robotino2 : public Com
{
private:
    /// Number of Robotino motors.
    const unsigned char motor_num = 3;

    /// Array of Robotino motors.
    std::array<Motor, 3> motor;

    /// Motors' actual positions in encoder ticks. 
    std::array<int, 3> actual_position;
    
    /// Motors' actual velocities in rad/s. 
    std::array<float, 3> actual_velocity;

    /// Motors' actual currents in A. 
    std::array<float, 3> actual_current;

    /// Represents a Robotino motion drive.
    OmniDrive omniDrive;

    /// Motor's velocity limit in rad/s.
    const float motor_vel_limit = 240;

    /// Robot's linear speed limit in m/s.
    const float robot_lin_speed_limit = 0.8f;

    /// Robot's angular velocity limit in rad/s.
    const float robot_vel_limit = static_cast<float>(PI);

public:
    /**
     * Constructor. Start connection with Robotino4.
     * 
     * @param ip_addr IP-address of Robotino4.
     * @param sample_time sample time of motors' control and sensors' measurements in ms. Default 20 ms.
     */
    Robotino2(const std::string& ip_addr, unsigned int sample_time=20);

    /**
     * Destructor. Close connection with Robotino.
     */
    ~Robotino2();

    /**
     * This function is called on errors.
     * 
     * @param error the error which caused this event.
     * @param errorString a human readable error description.
     * @remark This function is called from outside the applications main thread.
     * This is extremely important particularly with regard to GUI applications.
     */
	void errorEvent(Error error, const char* errorString);

    /**
     * This function is called if a connection to Robotino has been established.
     * 
     * @remark This function is called from outside the applications main thread. 
     * This is extremely important particularly with regard to GUI applications.
     */
	void connectedEvent();

    /**
     * This function is called when a connection is closed.
     * 
     * @remark This function is called from outside the applications main thread.
     * This is extremely important particularly with regard to GUI applications.
     */
	void connectionClosedEvent();

    /**
     * This function is called whenever sensor readings have been updated.
     */
    void updateEvent();

    /**
     * Encoder position of motor.
     * 
     * @param num motor number.
     * @return actual position of this motor in encoder ticks.
     */
    int get_actual_position(unsigned char num);

    /**
     * Encoder position of all motors.
     * 
     * @return actual positions of all motors in encoder ticks.
     */
    std::array<int, 3> get_actual_positions();

    /**
     * @param num motor number.
     * @return actual velocity of this motor in rad/s.
     */
    float get_actual_velocity(unsigned char num);

    /**
     * @return actual velocities of all motors in rad/s.
     */
    std::array<float, 3> get_actual_velocities();

    /**
     * @param num motor number.
     * @return current of this motor in A.
     */
    float get_actual_current(unsigned char num);

    /**
     * @return currents of all motor in A.
     */
    std::array<float, 3> get_actual_currents();

    /**
     * Sets the setpoint speed of this motor.
     * 
     * @param num motor number.
     * @param speed setpoint speed in rad/s.
     * @return True if motor set speed doesn't exceed the limit value and was set. Otherwise false.
     */
    bool set_motor_speed(unsigned char num, float speed);

    /**
     * Sets the setpoint speed of all motors.
     * 
     * @param speeds speed setpoints for all motors in rad/s.
     * @return True if motor set speeds don't exceed the limit value and were set. Otherwise false.
     */
    bool set_motor_speeds(const std::array<float, 3>& speeds);

    /**
     * Resets the position of this motor.
     * 
     * @param num motor number.
     */
    void reset_motor_position(unsigned char num);

    /**
     * Resets the position of all motors.
     */
    void reset_motor_positions();

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
    void set_motor_pid(unsigned char num, unsigned char kp, unsigned char ki, unsigned char kd);

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
    void set_motor_pids(const std::array<unsigned char, 3>& kp,
                        const std::array<unsigned char, 3>& ki,
                        const std::array<unsigned char, 3>& kd);

    /**
     * Set robot speed.
     * 
     * @param vx speed along x axis of robot's local coordinate system in m/s.
     * @param vy speed along y axis of robot's local coordinate system in m/s.
     * @param omega angular velocity of rotation in rad/s.
     * @remark This function is thread save. It should be called about every 100ms.
     * @return True if robot set speed doesn't exceed the limit value and were set. Otherwise false.
     */
    bool set_robot_speed(float vx, float vy, float omega);

    /**
     * Project the velocity of the robot in cartesian coordinates to single motor speeds.
     * 
     * @param vx speed along x axis of robot's local coordinate system in m/s.
     * @param vy speed along y axis of robot's local coordinate system in m/s.
     * @param omega angular velocity of rotation in rad/s.
     */
    std::array<float, 3> robot_speed_to_motor_speeds(float vx, float vy, float omega);
};

/**
 * Convert velocity from rpm to rad/s.
 * 
 * @param rpm velocity in rpm.
 * @return Velocity in rad/s.
*/
inline float rpm2rads(float rpm) {
    return 2.f * PI * rpm / 60.f;
}

/**
 * Convert velocity from rad/s to rpm.
 * 
 * @param rads velocity in rad/s.
 * @return Velocity in rpm.
*/
inline float rads2rpm(float rads) {
    return 60.f * rads / (2.f * PI);
}

#endif  // ROBOTINO2_HPP
