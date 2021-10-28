#include "robotino2.hpp"

void MyCom::errorEvent(Error error, const char* errorString)
{
    std::cerr << "Error: " << errorString << std::endl;
	exit(1);
}

void MyCom::connectedEvent()
{
    std::cout << "Connected." << std::endl;
}

void MyCom::connectionClosedEvent()
{
    std::cout << "Connection closed." << std::endl;
}

void MyCom::updateEvent()
{
    for (int i = 0; i < 3; i++) {
			actual_position[i] = motor[i].actualPosition();
			actual_velocity[i] = 2 * PI * motor[i].actualVelocity() / 60;
			actual_current[i] = motor[i].motorCurrent();
		}
}

int MyCom::get_actual_position(size_t num){
    return actual_position[num];
}

std::vector<int> MyCom::get_actual_positions()
{
    return actual_position;
}

float MyCom::get_actual_velocity(size_t num)
{
    return actual_velocity[num];
}

std::vector<float> MyCom::get_actual_velocities()
{
    return actual_velocity;
}

float MyCom::get_actual_current(size_t num)
{
    return actual_current[num];
}

std::vector<float> MyCom::get_actual_currents()
{
    return actual_current;
}



Robotino2::Robotino2(const std::string& ip_addr, unsigned int sample_time)
{
    omniDrive.setComId(com.id());
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setComId(com.id());
        motor[i].setMotorNumber(i);
    }

	// Connect
    std::cout << "Connecting...";
	com.setAddress( ip_addr.c_str() );
	com.connect();
	if( !com.isConnected() )
	{
		std::cout << std::endl << "Could not connect to " << com.address() << std::endl;
		exit( 1 );
	}
	else
	{
		std::cout << "success" << std::endl;
	}

    com.setMinimumUpdateCycleTime(sample_time);
	omniDrive.setVelocity(0, 0, 0);
}

Robotino2::~Robotino2()
{
    com.disconnect();
}

int Robotino2::get_actual_position(size_t num){
    return motor[num].actualPosition();
}

std::vector<int> Robotino2::get_actual_positions()
{
    std::vector<int> pos = {0, 0, 0};
    for (size_t i = 0; i < motor_num; i++)
    {
        pos[i] = motor[i].actualPosition();
    }
    
    return pos;
}

float Robotino2::get_actual_velocity(size_t num)
{
    return 2 * PI * motor[num].actualVelocity() / 60;
}

std::vector<float> Robotino2::get_actual_velocities()
{
    std::vector<float> vel = {0, 0, 0};
    for (size_t i = 0; i < motor_num; i++)
    {
        vel[i] = 2 * PI * motor[i].actualVelocity() / 60;
    }
    return vel;
}

float Robotino2::get_actual_current(size_t num)
{
    return motor[num].motorCurrent();
}

std::vector<float> Robotino2::get_actual_currents()
{
    std::vector<float> cur = {0, 0, 0};
    for (size_t i = 0; i < motor_num; i++)
    {
        cur[i] = motor[i].motorCurrent();
    }
    return cur;
}

void Robotino2::set_motor_speed(size_t num, float speed)
{
    motor[num].setSpeedSetPoint(60 * speed / (2 * PI));
}

void Robotino2::set_motors_speed(const std::vector<float>& speeds)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setSpeedSetPoint(60 * speeds[i] / (2 * PI));
    }
}

void Robotino2::reset_motor_position(size_t num)
{
    motor[num].resetPosition();
}

void Robotino2::reset_motors_position()
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].resetPosition();
    }
}

void Robotino2::set_motor_pid(size_t num, float kp, float ki, float kd)
{
    motor[num].setPID(kp, ki, kd);
}

void Robotino2::set_motors_pid(const std::vector<float>& kp, const std::vector<float>& ki, const std::vector<float>& kd)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setPID(kp[i], ki[i], kd[i]);
    }
    
}

void Robotino2::set_robot_speed(float vx, float vy, float omega)
{
    omniDrive.setVelocity(vx*1000.0, vy*1000.0, omega);
}