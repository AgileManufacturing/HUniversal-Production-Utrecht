/**
 * @file KeyBoardControlNode.h
 * @brief An ROS node for controlling the delta robot with the keyboard
 * @date Created: 2012-10-12
 *
 * @author ??
 * @author Alexander Streng
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

#pragma once

#include <ros/ros.h>
#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_knowledge_database/ModuleIdentifier.h>
#include <rexos_datatypes/InstructionData.h>
#include "rexos_logger/rexos_logger.h"
#include <vectors/Vectors.h>

namespace keyboard_control_node {

	class KeyBoardControlNode 
	{
	public:
		KeyBoardControlNode(std::string blackboardIp, std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier);
		~KeyBoardControlNode();
		void run();
		
	private:
		void readInputFromKeyBoard(int inputCharacter);
		void writeToBlackBoard(Vector3 direction, double acceleration, double rotationX, double rotationY, double rotationZ);
		
		/**
		 * @var double maxAcceleration
		 * The maxAcceleration of the effector in millimeters per second.
		 **/
		double maxAcceleration;
		static const char KEYCODE_W = 0x77;
		static const char KEYCODE_S = 0x73;
		static const char KEYCODE_A = 0x61;
		static const char KEYCODE_D = 0x64;
		static const char KEYCODE_UP = 0x41;
		static const char KEYCODE_DOWN = 0x42;
		static const char KEYCODE_J = 0x6A;
		static const char KEYCODE_L = 0x6C;
		static const char KEYCODE_I = 0x69;
		static const char KEYCODE_K = 0x6B;
		static const char KEYCODE_U = 0x75;
		static const char KEYCODE_O = 0x6F;
		static const char KEYCODE_Q = 0x71;
		
		/**
		 * @var int keyboardNumber
		 * The number of the keyboard, e.g.: 0 is the primary keyboard.
		 **/
		static const int KEYBOARDNUMBER = 0;

		/**
		 * @var double step
		 * The size in millimeters per movement.
		 **/
		static constexpr double STEP = 1.0;
		static constexpr double STEP_ANGLE = 0.1;

		/**
		 * A terminal interface data struct.
		 **/
		struct termios oldTerminalSettings, newTerminalSettings;
		Blackboard::BlackboardCppClient* equipletStepBlackboardClient;
		bool exitProgram;
		rexos_knowledge_database::ModuleIdentifier identifier;
		std::string equipletName;
	};

}
