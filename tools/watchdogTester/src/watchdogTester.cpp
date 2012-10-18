#include <iostream>
#include <InputOutput/OutputDevices/Gripper.h>

void error(){
	std::cout << "Gripper handler warning!" << std::endl;
}

/**
 *
 */
int main(void){
	InputOutput::OutputDevices::Gripper grp(error);
	std::cout << "Welcome to the gripper watchdog tester!" <<  std::endl;
	std::cout << "Keys to controll the gripper" << std::endl;
	std::cout << "0\tTurn gripper off" << std::endl;
	std::cout << "1\tTurn gripper on" << std::endl;
	std::cout << "-1\t Exit program" << std::endl;

	int input;
	do {
		std::cin >> input;
		if(input == 1){
			grp.grab();
		} else if(input == 0){
			grp.release();
		}
	} while(input != -1);

	return 0;
}