#include <ros_for_can.h>

int main(int argc, char const *argv[])
{
	CanMsg message = CanMsg(1, 2, 0x00);
	message.set(0x48, 0x69);


	ros_for_can rfc = ros_for_can("/dev/pcan32", 115200);

	rfc.init_ret();
	while(1){
		bool flag = rfc.transmitMsg(message);
		if (flag==1)
			message.print();
	}
	return 0;
}
