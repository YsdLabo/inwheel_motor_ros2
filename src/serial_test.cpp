#include<rclcpp/rclcpp.hpp>
#include"serial_controller.h"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	// Start
	SerialPort sp;
	sp.setDevice("/dev/ttyAMA1");
	sp.setBaudrate(115200);
	sp.Open();
	
	// Write Command
	char cmd[] = {0x02, 0x70, 0x05, 0x09, 0x03};  // STX,to_CPU_ID,Num,Mode,ETX
	sp.Flush();
	sp.Write(cmd, 5);  // Write 5 bytes
	
	// Read Command
	char res[9];
	sp.Read(res, 9);
/*	int num=0, cnt=0;
	while(num != 9) {  // Read 9 bytes
		char tmp;
		if(sp.Read(&tmp, 1) == 1) {
			res[num] = tmp;
			num ++;
			cnt = 0;
			continue;
		}
		// Time out
		if(cnt++ == 10) {
			sp.Flush();
			printf("Time out\n");
			break;
		}
	}
*/
	printf("Motor Power: %d\n", res[5]);
	printf("Batt Voltage: %f [V]\n", res[6]*0.04);
	printf("Batt Current: %f [A]\n", res[7]*0.1);

	// Finish
	sp.Close();
	rclcpp::shutdown();

	return 0;
}
