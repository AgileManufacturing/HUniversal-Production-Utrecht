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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

public class GripperModule implements Module{
	int gripperSize = 2;
	Module movementModule;
	int id;
	HashMap<Integer, Object> configuration;
	
	public GripperModule(HashMap<Integer, Object> configuration, Integer id){
		this.configuration = configuration;
		this.id = id;
	}

	@Override
	public EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters) {		
		EquipletStepMessage[] equipletSteps;
		ArrayList<EquipletStepMessage> steps;
		switch(stepType){
		
		case 1: // pick
			steps = new ArrayList<EquipletStepMessage>();
			
			BasicDBObject moveParameters = (BasicDBObject) parameters.copy();
			moveParameters.put("extraSize", gripperSize);
			steps.addAll(new ArrayList<EquipletStepMessage>
				(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters))));//MOVE TO
			
			InstructionData instructionData = new InstructionData("deactivate", "gripper", "FIND_ID",
					new BasicDBObject("ID", ((BasicDBObject) parameters.get("position")).get("relativeToPart")),
					new BasicDBObject());
			steps.add(new EquipletStepMessage(null, instructionData,
					StepStatusCode.EVALUATING, new TimeData(1)));//ACTIVATE GRIPPER
			
			steps.addAll(new ArrayList<EquipletStepMessage>
				(Arrays.asList(movementModule.getEquipletSteps(2, parameters))));//SAVE MOVE
			equipletSteps = new EquipletStepMessage[steps.size()];
			return steps.toArray(equipletSteps);
		case 2: // place/drop
			steps = new ArrayList<EquipletStepMessage>();
			steps.addAll(new ArrayList<EquipletStepMessage>
				(Arrays.asList(movementModule.getEquipletSteps(1, parameters))));//MOVE TO
			steps.add(new EquipletStepMessage(null, new InstructionData(),
					StepStatusCode.EVALUATING, new TimeData(1)));//DEACTIVATE GRIPPER
			steps.addAll(new ArrayList<EquipletStepMessage>
				(Arrays.asList(movementModule.getEquipletSteps(2, parameters))));//SAVE MOVE
			equipletSteps = new EquipletStepMessage[steps.size()];
			return steps.toArray(equipletSteps);
		default:
			break;
		
		}
		return new EquipletStepMessage[0];
	}

	@Override
	public int[] isLeadingForSteps() {
		
		int[]steps = {1,2,3,4,5};
		
		return steps;
	}
}
