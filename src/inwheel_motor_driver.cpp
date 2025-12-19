#include "inwheel_motor_driver.hpp"

InwheelMotorDriver::InwheelMotorDriver(const char* _device)
{
	// initialize serial 
	sp.setDevice(_device);
	printf("[InwheelMotorDriver] set device: %s\n", _device);
	sp.setBaudrate(BAUDRATE);
	printf("[InwheelMotorDriver] set baudrate: %d bps\n", BAUDRATE);
	sp.Open();
	printf("[InwheelMotorDriver] serial port open\n");

	set_acceleration(2);
	set_deacceleration(0.5);
	// Drive mode
	unsigned char cmd[10] = {0x01, 0x51, 0x70, 0x17, 0, 0, 0, 0, 0x03, 0xDC};
	unsigned char res[10];
	WriteCommand(cmd, res);
	printf("[InwheelMotorDriver] initialize finished\n");
}

InwheelMotorDriver::~InwheelMotorDriver()
{
	close();
}

void InwheelMotorDriver::close()
{
	motor_stop();
	usleep(50000);
	motor_off();
	usleep(50000);
	sp.Close();
}

void InwheelMotorDriver::motor_on()
{
	unsigned char cmd[10] = {0x01, 0x52, 0x70, 0x19, 0, 0, 0, 0, 0x0F, 0xEB};
	unsigned char res[10];
	WriteCommand(cmd, res);
	printf("[InwheelMotorDriver] motor on\n");
}

void InwheelMotorDriver::motor_off()
{
	unsigned char cmd[10] = {0x01, 0x52, 0x70, 0x19, 0, 0, 0, 0, 0x06, 0xE2};
	unsigned char res[10];
	WriteCommand(cmd, res);
	printf("[InwheelMotorDriver] motor off\n");
}

void InwheelMotorDriver::motor_free()
{
	motor_off();
}

void InwheelMotorDriver::move(double rpm)
{
	int dec = (int)(rpm * 512.0 * PulsePerRotate / 1875.0);
	unsigned char cmd[10] = {0x01, 0x54, 0x70, 0xB2, 0};
	cmd[5] = (char)((dec & 0xFF000000) >> 24);
	cmd[6] = (char)((dec & 0x00FF0000) >> 16);
	cmd[7] = (char)((dec & 0x0000FF00) >> 8);
	cmd[8] = (char)(dec & 0x000000FF);
	checksum(cmd);

	unsigned char res[10];
	WriteCommand(cmd, res);
}

void InwheelMotorDriver::motor_stop()
{
	move(0);
}

void InwheelMotorDriver::set_acceleration(double rpss)
{
	unsigned char cmd[10] = {0x01, 0x54, 0x70, 0x99, 0};
	unsigned char res[10];
	acc = rpss;
	int dec = (int)(acc * 256.0 * PulsePerRotate / 15625.0);
	cmd[5] = (char)((dec & 0xFF000000) >> 24);
	cmd[6] = (char)((dec & 0x00FF0000) >> 16);
	cmd[7] = (char)((dec & 0x0000FF00) >> 8);
	cmd[8] = (char)(dec & 0x000000FF);
	checksum(cmd);

	WriteCommand(cmd, res);
}

void InwheelMotorDriver::set_deacceleration(double rpss)
{
	unsigned char cmd[10] = {0x01, 0x54, 0x70, 0x9A, 0};
	unsigned char res[10];
	deacc = rpss;
	int dec = (int)(deacc * 256.0 * PulsePerRotate / 15625.0);
	cmd[5] = (char)((dec & 0xFF000000) >> 24);
	cmd[6] = (char)((dec & 0x00FF0000) >> 16);
	cmd[7] = (char)((dec & 0x0000FF00) >> 8);
	cmd[8] = (char)(dec & 0x000000FF);
	checksum(cmd);

	WriteCommand(cmd, res);
}

int InwheelMotorDriver::get_pulse_count(int* count)
{
	unsigned char cmd[10] = {0x01, 0xA0, 0x70, 0x71, 0, 0, 0, 0, 0, 0x82};
	unsigned char res[10];

	int ret = WriteCommand(cmd, res);

	if(ret == 0) {
		int tmp = 0;
		tmp |= (int)res[5] << 24;
		tmp |= (int)res[6] << 16;
		tmp |= (int)res[7] << 8;
		tmp |= res[8];
		*count = tmp;
		return 0;    // success
	}
	return ret;    // failed
}

void InwheelMotorDriver::reset_pulse_count()
{
	motor_off();
	
	unsigned char cmd_reset[10] = {0x01, 0x51, 0x70, 0xAC, 0, 0, 0, 0, 1, 0x6F};
	unsigned char res[10];
	WriteCommand(cmd_reset, res);
	printf("[InwheelMotorDriver] reset encoder\n");
	
	motor_on();
}

int InwheelMotorDriver::WriteCommand(unsigned char cmd[10], unsigned char res[10])
{
	sp.Flush();
	sp.Write((const char*)cmd, 10);

	int num = 0, cnt = 0;

	while(num != 10) {
		char tmp;
		if(sp.Read(&tmp, 1) == 1) {
			res[num] = tmp;
			num ++;
			cnt = 0;
		}
		// Time out
		else {
			usleep(1000);
			if(cnt++ == 100) {
				sp.Flush();
				return -1;
			}
		}
	}

	// Response Check
	if(res[0] != cmd[0]) return -2;
	if((cmd[1]&0xF0)==0x50 && res[1] != cmd[1] + 0x10) return -3;
	else if(cmd[1]==0xA0 && cmd[1] != (res[1]&0xF0)) return -3;
	if(res[2] != cmd[2]) return -4;
	if(res[3] != cmd[3]) return -5;
	if(res[4] != 0) return res[4];

	return 0; // success
}

void InwheelMotorDriver::checksum(unsigned char cmd[10])
{
	int buf = 0;
	for(int i=0;i<9;i++) buf += cmd[i];
	cmd[9] = (char)(buf & 0xFF);
}


