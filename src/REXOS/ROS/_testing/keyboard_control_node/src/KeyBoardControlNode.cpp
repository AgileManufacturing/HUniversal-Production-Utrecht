/**
 * @file KeyBoardControlNode.cpp
 * @brief Control the robot with the WASD UP and Down keys.
 * @date Created: 2014-11-05
 *
 * @author Dick van der Steen
 * @author Dennis Koole
 * @author Tommas Bakker
 * @author Peter Markotic
 *
 * @section LICENSE
 * License: newBSD
 * 
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <cstdlib>
#include <keyboard_control_node/KeyBoardControlNode.h>
#include "rexos_utilities/Utilities.h"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>
#include <rexos_datatypes/EquipletStep.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "KeyBoardControlNode"
// @endcond

using namespace keyboard_control_node;


KeyBoardControlNode::KeyBoardControlNode(std::string blackboardIp, std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier) :
	maxAcceleration(5.0), exitProgram(false), equipletName(equipletName), identifier(moduleIdentifier) 
{
	REXOS_INFO("Constructing");

	//DirectMoveStepsBlackBoard
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, equipletName, "EquipletStepsBlackBoard");

	tcgetattr(KEYBOARDNUMBER, &oldTerminalSettings);
	memcpy(&newTerminalSettings, &oldTerminalSettings, sizeof(struct termios));
	newTerminalSettings.c_lflag &=~ (ICANON | ECHO);

	// Setting a new line, then end of file.
	newTerminalSettings.c_cc[VEOL] = 1;
	newTerminalSettings.c_cc[VEOF] = 2;
	tcsetattr(KEYBOARDNUMBER, TCSANOW, &newTerminalSettings);

	REXOS_INFO("Reading from keyboard");
	REXOS_INFO_STREAM("Start controlling the robot by pressing the following keys:" << std::endl <<
			"KEY   ACTION                    MUTATION" << std::endl <<
			"-----------------------------------------" << std::endl <<
			"W     Forward                   MoveY++" << std::endl <<
			"D     Backward                  MoveY--" << std::endl <<
			"A     Left                      MoveX--" << std::endl <<
			"D     Right                     MoveX++" << std::endl <<
			"UP    Up                        MoveZ++" << std::endl <<
			"DOWN  Down                      MoveZ--" << std::endl <<
			"J     Rorate to the left        RotateX++" << std::endl <<
			"L     Rorate to the right       RotateX--" << std::endl <<
			"I     Rorate to the front       RotateY++" << std::endl <<
			"K     Rorate to the back        RotateY++" << std::endl <<
			"U     Rotate clockwise          RotateZ++" << std::endl <<
			"O     Rotate counter clockwise  RotateZ--" << std::endl <<
			"X     Run preprogrammed Demo" << std::endl <<
			"Q     Quit program");
}

KeyBoardControlNode::~KeyBoardControlNode() {
	REXOS_INFO("Destructing");
	tcsetattr(KEYBOARDNUMBER, TCSANOW, &oldTerminalSettings);
}

void KeyBoardControlNode::run() {
	while(exitProgram == false) {
		char inputCharacter;
		
		// Get the next event from the keyboard.
		if(read(KEYBOARDNUMBER, &inputCharacter, 1) < 0){
			perror("read():");
			exit(1);
		}
		readInputFromKeyBoard(inputCharacter);
	}
}

void KeyBoardControlNode::readInputFromKeyBoard(int inputCharacter){
	// Reads keyboard input and does things. Doesnt keep track of actual robot location, just relative
	Vector3 direction;
	double rotationX = 0, rotationY = 0, rotationZ = 0;
	bool sendMessage = false;
	
	// Check which key was pressed.
	switch(inputCharacter){
		case KEYCODE_X:
			REXOS_INFO("PRESSED X");
			this->playRoutine();
			REXOS_INFO("routuneplayed.asdasdjhbaskdjnsa");
		break;
		case KEYCODE_W:
			REXOS_INFO("PRESSED W");
			direction.y+= STEP;
			sendMessage = true;
		break;
		case KEYCODE_A:
			REXOS_INFO("PRESSED A");
			direction.x-= STEP;
			sendMessage = true;
		break;
		case KEYCODE_S:
			REXOS_INFO("PRESSED S");
			direction.y-= STEP;
			sendMessage = true;
		break;
		case KEYCODE_D:
			REXOS_INFO("PRESSED D");
			direction.x+= STEP;
			sendMessage = true;
		break;
		case KEYCODE_UP:
			REXOS_INFO("PRESSED UP");
			direction.z+= STEP;
			sendMessage = true;
		break;
		case KEYCODE_DOWN:
			REXOS_INFO("PRESSED DOWN");
			direction.z-= STEP;
			sendMessage = true;
		break;
		
		case KEYCODE_J:
			REXOS_INFO("PRESSED J");
			rotationY -= STEP_ANGLE;
			sendMessage = true;
		break;
		case KEYCODE_L:
			REXOS_INFO("PRESSED L");
			rotationY += STEP_ANGLE;
			sendMessage = true;
		break;
		case KEYCODE_I:
			REXOS_INFO("PRESSED I");
			rotationX -= STEP_ANGLE;
			sendMessage = true;
		break;
		case KEYCODE_K:
			REXOS_INFO("PRESSED K");
			rotationX += STEP_ANGLE;
			sendMessage = true;
		break;
		case KEYCODE_U:
			REXOS_INFO("PRESSED U");
			rotationZ += STEP_ANGLE;
			sendMessage = true;
		break;
		case KEYCODE_O:
			REXOS_INFO("PRESSED O");
			rotationZ -= STEP_ANGLE;
			sendMessage = true;
		break;
		
		case KEYCODE_Q:
			exitProgram = true;
			break;
		default:
			REXOS_INFO_STREAM("Unknown character: " << inputCharacter);
		break;
			
	}
	if(sendMessage == true) {
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
	}
}

void KeyBoardControlNode::writeToBlackBoard(Vector3 direction, double acceleration, double rotationX, double rotationY, double rotationZ){
	rexos_datatypes::EquipletStep equipletStep;
	Json::Value instructionData;
	
	Json::Value moveCommand;
	moveCommand["x"] = direction.x;
	moveCommand["y"] = direction.y;
	moveCommand["z"] = direction.z;
	moveCommand["maxAcceleration"] = 5.0;
	instructionData["move"] = moveCommand;
	
	Json::Value rotateCommand;
	rotateCommand["x"] = rotationX;
	rotateCommand["y"] = rotationY;
	rotateCommand["z"] = rotationZ;
	instructionData["rotate"] = rotateCommand;
	
	equipletStep.setInstructionData(instructionData);
	
	rexos_datatypes::OriginPlacement originPlacement;
	originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::OriginPlacementType::RELATIVE_TO_CURRENT_POSITION);
	equipletStep.setOriginPlacement(originPlacement);
	
	equipletStep.setModuleIdentifier(identifier);
	equipletStep.setStatus("WAITING");
	
	REXOS_INFO_STREAM("hardware step: " << equipletStep.toJSON());
	Json::StyledWriter writer;
	if(equipletStepBlackboardClient->insertDocument(writer.write(equipletStep.toJSON())) == false) {
		REXOS_WARN("insertion of hardware step on blackboard failed!");
	}
}

void KeyBoardControlNode::playRoutine(){
	REXOS_INFO("IN DE FUNCITE");
	Vector3 direction;
	double rotationX = 0, rotationY = 0, rotationZ = 0;
	
	while(true){
		/*direction.z = 15;
		REXOS_INFO("OMHOOG1");
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		REXOS_INFO("OMHOOG2");
		ros::Duration(0.5).sleep();
		REXOS_INFO("OMHOOG3");
		
		
		direction.z = -30;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
	

		direction.z = 15;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
		direction.z = 0;
		
		
		direction.y = 15;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
	
		
		direction.y = -30;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
		
		
		direction.y = 15;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
		direction.y = 0;
		
		
		direction.x = 15;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
		
		
		direction.x = -30;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
	
		
		direction.x = 15;
		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(0.5).sleep();
		direction.x = 0;
		
		//This is the above code but then in less rules
		for(int i = 0; i < 9; ++i;){
			int j = (((i * 5)%10) + 5);
			
			if(i < 3){
				if(i % 2 == 0){
					direction.z -= j;
				} else direction.z += j;
			} else if(i < 6){
				if(i % 2 == 0){
					direction.y -= j;
				} else direction.y += j;
			} else {
				if(i % 2 == 0){
					direction.x -= j;
				} else direction.x += j;
			}	
			
			writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
			ros::sleep(1000);
			std::cin.peek() != EOF;
		}*/
		//C 2 north//

		direction.y = 40;
		direction.x = 0;

		rotationX = 0.2;//0.2
		rotationY = 0;

		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(5).sleep();
		
		//2northeast//

		direction.y = -(40 - 20 * std::sqrt(2));//1
		direction.x = (20 * std::sqrt(2));
		
		rotationX = -0.1;//0.1	
		rotationY = 0.1;//0.1

		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(5).sleep();

		//2east//
		direction.y = -(20 * std::sqrt(2));//3
                direction.x = (40 - 20 * std::sqrt(2));

		rotationX = -0.1;//0
		rotationY = 0.1;//0.2

                writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
                ros::Duration(5).sleep();


		//2southeast//
		direction.y = -(20 * std::sqrt(2));//5 
		direction.x = -(40 - 20 * std::sqrt(2));

		rotationX = -0.1;//-0.1
		rotationY = -0.1;//0.1

		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(5).sleep();

		//2south//
                direction.y = -(40 - 20 * std::sqrt(2));//7
		direction.x = -(20 * std::sqrt(2));

		rotationX = -0.1;//-0.2	
		rotationY = -0.1;//0

                writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
                ros::Duration(5).sleep();
		
		//2southwest//	
		direction.y = (40 - 20 * std::sqrt(2));//9
		direction.x = -(20 * std::sqrt(2));

		rotationX = 0.1;//-0.1
		rotationY = -0.1;//-0.1

                writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
                ros::Duration(5).sleep();

		//2west//
		direction.y = (20 * std::sqrt(2));//11
                direction.x = -(40 - 20 * std::sqrt(2));
		
		rotationX = 0.1;//0
		rotationY = -0.1;//-0.2
		
                writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
                ros::Duration(5).sleep();

		//2northwest//
		direction.y = (20 * std::sqrt(2));//13
                direction.x = (40 - 20 * std::sqrt(2));
		
		rotationX = 0.1;//0.1	
		rotationY = 0.1;//-0.1

                writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
                ros::Duration(5).sleep();

		//2north//
		direction.y = (40 - 20 * std::sqrt(2));//15
                direction.x = (20 * std::sqrt(2));

		rotationX = 0.1;//0.2
		rotationY = 0.1;//0	

		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(5).sleep();

		//2centre//
		direction.y = -40;//17
                direction.x = 0;

		rotationX = -0.2;//0	
		rotationY = 0;//0		

		writeToBlackBoard(direction, maxAcceleration, rotationX, rotationY, rotationZ);
		ros::Duration(5).sleep();
	} 
}

/**
* Starting method for the KeyBoardControlNode.
*
* @param argc Argument count.
* @param argv Argument vector.
* 
* @return 1 on keyboard read error.
**/
int main(int argc, char** argv){
	// Ros init.
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	KeyBoardControlNode keyBoardControlNode("10.0.1.66", "EQ3", 
			rexos_knowledge_database::ModuleIdentifier("HU", "six_axis_type_A", "1"));
	keyBoardControlNode.run();
	
	return 0;
}
