/**
 * @file GripperModule.java
 * @brief Provides the data to be used for hardware agents.
 * @date Created: 12-04-13
 *
 * @author Thierry Gerritse
 * 
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
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

package rexos.mas.hardware_agent;

import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

public class GripperModule implements Module{
	
	public GripperModule(){
	}
		
	@Override
	public EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		TimeData td = new TimeData(2); //TODO: timedata hardcoded
		
		EquipletStepMessage[] equipletSteps = new EquipletStepMessage[3];
		
		switch(stepType){
		
		case 1: // pick
			equipletSteps[0] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(3));
			equipletSteps[1] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(4));
			equipletSteps[2] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(5));
			break;
		
		case 2: // place/drop
			equipletSteps[0] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(2));
			equipletSteps[1] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(3));
			equipletSteps[2] = new EquipletStepMessage(null, new InstructionData(), StepStatusCode.EVALUATING, new TimeData(4));
			break;
		
		}
		
		
		return equipletSteps;
	}

	@Override
	public int[] isLeadingForSteps() {
		
		return null;
	}
}
