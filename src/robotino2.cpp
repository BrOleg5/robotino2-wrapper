#include "robotino2.hpp"

Robotino2::Robotino2(const std::string& ip_addr, unsigned int sample_time): Com(), 
                                                                            actual_position({0, 0, 0}),
                                                                            actual_velocity({0.f, 0.f, 0.f}),
                                                                            actual_current({0.f, 0.f, 0.f})
{
    omniDrive.setComId(this->id());
    for (unsigned char i = 0; i < motor_num; i++)
    {
        motor[i].setComId(this->id());
        motor[i].setMotorNumber(i);
    }

	// Connect
    std::cout << "Connecting...";
	this->setAddress( ip_addr.c_str() );
	this->connect();
	if(!this->isConnected()) {
		std::cout << '\n' << "Could not connect to " << this->address() << '\n';
		exit(1);
	}
	else {
		std::cout << "success" << '\n';
	}

    this->setMinimumUpdateCycleTime(sample_time);
	omniDrive.setVelocity(0, 0, 0);
}

Robotino2::~Robotino2() {
    this->disconnect();
}

void Robotino2::errorEvent(Error error, const char* errorString) {
    std::cerr << "Error: " << errorString << std::endl;
	exit(1);
}

void Robotino2::connectedEvent() {
    std::cout << "Connected." << std::endl;
}

void Robotino2::connectionClosedEvent() {
    std::cout << "Connection closed." << std::endl;
}

void Robotino2::updateEvent() {
    for (int i = 0; i < 3; i++) {
			actual_position[i] = motor[i].actualPosition();
			actual_velocity[i] = rpm2rads(motor[i].actualVelocity());
			actual_current[i] = motor[i].motorCurrent();
		}
}

int Robotino2::get_actual_position(unsigned char num) {
    return actual_position[num];
}

std::array<int, 3> Robotino2::get_actual_positions() {
    return actual_position;
}

float Robotino2::get_actual_velocity(unsigned char num) {
    return actual_velocity[num];
}

std::array<float, 3> Robotino2::get_actual_velocities() {
    return actual_velocity;
}

float Robotino2::get_actual_current(unsigned char num) {
    return actual_current[num];
}

std::array<float, 3> Robotino2::get_actual_currents() {
    return actual_current;
}

bool Robotino2::set_motor_speed(unsigned char num, float speed) {
    if(std::abs(speed) <= motor_vel_limit) {
        motor[num].setSpeedSetPoint(rads2rpm(speed));
        return true;
    }
    else {
        return false;
    }
}

bool Robotino2::set_motor_speeds(const std::array<float, 3>& speeds) {
    if((std::abs(speeds[0]) <= motor_vel_limit) &&
       (std::abs(speeds[1]) <= motor_vel_limit) &&
       (std::abs(speeds[2]) <= motor_vel_limit)) {
        for (unsigned char i = 0; i < motor_num; i++) {
            motor[i].setSpeedSetPoint(rads2rpm(speeds[i]));
        }
        return true;
    }
    else {
        return false;
    }
}

void Robotino2::reset_motor_position(unsigned char num) {
    motor[num].resetPosition();
}

void Robotino2::reset_motor_positions() {
    for (unsigned char i = 0; i < motor_num; i++)
    {
        motor[i].resetPosition();
    }
}

void Robotino2::set_motor_pid(unsigned char num, unsigned char kp, unsigned char ki, unsigned char kd) {
    motor[num].setPID(kp, ki, kd);
}

void Robotino2::set_motor_pids(const std::array<unsigned char, 3>& kp,
                               const std::array<unsigned char, 3>& ki,
                               const std::array<unsigned char, 3>& kd) {
    for (unsigned char i = 0; i < motor_num; i++) {
        motor[i].setPID(kp[i], ki[i], kd[i]);
    }
    
}

bool Robotino2::set_robot_speed(float vx, float vy, float omega) {
    if((std::abs(vx) <= robot_lin_speed_limit) &&
       (std::abs(vy) <= robot_lin_speed_limit) &&
       (std::abs(omega) <= robot_vel_limit)) {
        omniDrive.setVelocity(vx*1000.f, vy*1000.f, 180.f*omega/PI);
        return true;
    }
    else {
        return false;
    }
}

std::array<float, 3> Robotino2::robot_speed_to_motor_speeds(float vx, float vy, float omega) {
    float m1, m2, m3;
    omniDrive.project(&m1, &m2, &m3, vx*1000, vy*1000, 180.f*omega/PI);
    std::array<float, 3> res = {m1, m2, m3};
    for (unsigned char i = 0; i < 3; i++) {
        res[i] = rpm2rads(res[i]);
    }
    return res;
}
