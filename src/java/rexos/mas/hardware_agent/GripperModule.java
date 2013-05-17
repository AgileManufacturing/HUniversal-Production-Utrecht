/**
 * @file GripperModule.java
 * @brief Provides the data to be used for hardware agents.
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/

package rexos.mas.hardware_agent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import rexos.mas.data.Position;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

public class GripperModule extends Module {
	private static final int GRIPPER_SIZE = 2;

	private Module movementModule;

	public GripperModule() {
	}

	@Override
	public EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		EquipletStepMessage[] equipletSteps;
		ArrayList<EquipletStepMessage> steps;

		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		switch (stepType) {

		case 1: // pick
			steps = new ArrayList<EquipletStepMessage>();

			BasicDBObject moveParameters = (BasicDBObject) parameters.copy();
			moveParameters.put("extraSize", GRIPPER_SIZE);

			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));// SAVE MOVE
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, moveParameters)));// MOVE TO XY
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, moveParameters)));// MOVE TO Z
			steps.add(activateGripper(moveParameters));// ACTIVATE GRIPPER
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));// SAVE MOVE

			equipletSteps = new EquipletStepMessage[steps.size()];
			return steps.toArray(equipletSteps);
		case 2: // place
			steps = new ArrayList<EquipletStepMessage>();

			moveParameters = (BasicDBObject) parameters.copy();
			moveParameters.put("extraSize", GRIPPER_SIZE);

			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));// SAVE MOVE
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, moveParameters)));// MOVE TO XY
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, moveParameters)));// MOVE TO Z
			steps.add(deactivateGripper(moveParameters));// DEACTIVATE GRIPPER
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));// SAVE MOVE

			equipletSteps = new EquipletStepMessage[steps.size()];
			return steps.toArray(equipletSteps);
		default:
			break;

		}
		return new EquipletStepMessage[0];
	}

	@Override
	public EquipletStepMessage[] fillPlaceHolders(EquipletStepMessage[] steps, BasicDBObject parameters) {
		Position position = new Position((BasicDBObject) parameters.get("position"));

		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		
		ArrayList<EquipletStepMessage> movementModuleSteps = new ArrayList<EquipletStepMessage>();
		for (EquipletStepMessage step : steps) {
			if (step.getModuleId() == movementModule.getId()){
				movementModuleSteps.add(step);
			} else {
				InstructionData instructionData = step.getInstructionData();
				BasicDBObject lookUpParameters = instructionData.getLookUpParameters();
				if (lookUpParameters.getString("ID").equals("RELATIVE-TO-PLACEHOLDER")) {
					lookUpParameters.put("ID", position.getRelativeToPart());
				}
			}
		}
		EquipletStepMessage[] temp = new EquipletStepMessage[movementModuleSteps.size()];
		movementModule.fillPlaceHolders(movementModuleSteps.toArray(temp), parameters);
		return steps;
	}

	@Override
	public int[] isLeadingForSteps() {
		int[] steps = { 1, 2 };
		return steps;
	}

	private int findMovementModule(HashMap<Integer, Object> hashMap) {
		if (hashMap.containsKey(getId())) {
			return -1;
		}
		for (int key : hashMap.keySet()) {
			try {
				HashMap<Integer, Object> tempHashMap = (HashMap<Integer, Object>) hashMap.get(key);
				if (tempHashMap.containsKey(getId())) {
					return key;
				}
				int tempId = findMovementModule(tempHashMap);
				if (tempId != -1) {
					return tempId;
				}
			} catch (Exception e) {/* its no HashMap so do nothing */
			}
		}
		return -1;
	}

	private EquipletStepMessage activateGripper(BasicDBObject parameters) {
		Position position = new Position((BasicDBObject) parameters.get("Position"));
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}

		InstructionData instructionData = new InstructionData("activate", "gripper", "FIND_ID", lookUpParameters, new BasicDBObject());
		return new EquipletStepMessage(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(1));
	}

	private EquipletStepMessage deactivateGripper(BasicDBObject parameters) {
		System.out.println(parameters.toString());
		Position position = new Position((BasicDBObject) parameters.get("Position"));
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}

		InstructionData instructionData = new InstructionData("deactivate", "gripper", "FIND_ID", lookUpParameters, new BasicDBObject());
		return new EquipletStepMessage(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(1));
	}

}
