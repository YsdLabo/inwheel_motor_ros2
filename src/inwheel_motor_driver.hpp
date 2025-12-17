#ifndef  _INWHEEL_MOTOR_DRIVER_H_
#define  _INWHEEL_MOTOR_DRIVER_H_

#include<math.h>
#include<unistd.h>
#include"serial_controller.h"

#define BAUDRATE  115200
#define PulsePerRotate  4096.0

class InwheelMotorDriver
{
private:
	SerialPort sp;
	std::string _device;

	void checksum(unsigned char cmd[10]);
	int WriteCommand(unsigned char cmd[10], unsigned char res[10]);

	float acc;
	float deacc;
	//double wheel_radius = WHEEL_RADIUS;

public:
	InwheelMotorDriver(const char* _device);
	~InwheelMotorDriver();

	void close();
	void motor_on();
	void motor_off();
	void motor_free();
	void move(double rpm);
	void motor_stop();
	void set_acceleration(double rpss);
	void set_deacceleration(double rpss);
	int get_pulse_count(int* count);
	void reset_pulse_count();
};

#endif
