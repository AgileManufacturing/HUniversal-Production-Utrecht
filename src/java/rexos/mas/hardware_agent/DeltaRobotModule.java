/**
 * @file DeltaRobotModule.java
 * @brief Provides a deltaRobotModule.
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
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

import rexos.mas.data.Position;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

public class DeltaRobotModule extends Module {
	private static final double SAFE_MOVEMENT_PLANE = 6;

	public DeltaRobotModule() {
	}

	@Override
	public EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		switch (stepType) {
		case 1: // Move to savePlane
			return new EquipletStepMessage[] { moveToSafePlane(parameters) };
		case 2: // MoveXY
			return new EquipletStepMessage[] { moveXY(parameters) };
		case 3: // MoveZ
			return new EquipletStepMessage[] { moveZ(parameters) };
		default:
			break;
		}
		return new EquipletStepMessage[0];
	}

	@Override
	public EquipletStepMessage[] fillPlaceHolders(EquipletStepMessage[] steps, BasicDBObject parameters) {
		Position position = new Position((BasicDBObject) parameters.get("position"));
		for (EquipletStepMessage step : steps) {
			InstructionData instructionData = step.getInstructionData();
			BasicDBObject lookUpParameters = instructionData.getLookUpParameters();
			BasicDBObject payload = instructionData.getPayload();
			if (lookUpParameters.getString("ID").equals("RELATIVE-TO-PLACEHOLDER")) {
				lookUpParameters.put("ID", position.getRelativeToPart());
			}
			if (payload.getString("x").equals("X-PLACEHOLDER")) {
				payload.put("x", position.getX());
			}
			if (payload.getString("y").equals("Y-PLACEHOLDER")) {
				payload.put("y", position.getY());
			}
			if (payload.getString("z").equals("Z-PLACEHOLDER")) {
				payload.put("z", position.getZ());
			}
		}
		return steps;
	}

	@Override
	public int[] isLeadingForSteps() {
		return new int[] {};
	}

	private EquipletStepMessage moveToSafePlane(BasicDBObject parameters) {
		double extraSize = parameters.getDouble("extraSize");
		
		System.out.println(parameters.toString());
		
		BasicDBObject lookUpParameters = new BasicDBObject("ID", ((BasicDBObject) parameters.get("Position")).get("relativeToPart"));
		BasicDBObject payload = new BasicDBObject("z", extraSize + SAFE_MOVEMENT_PLANE);
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		EquipletStepMessage step = new EquipletStepMessage(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
		return step;
	}

	private EquipletStepMessage moveXY(BasicDBObject parameters) {
		Position position = new Position((BasicDBObject) parameters.get("Position"));
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}
		BasicDBObject payload = new BasicDBObject();
		if (position.getX() == -1) {
			payload.put("x", "X-PLACEHOLDER");
		} else {
			payload.put("x", position.getX());
		}
		if (position.getY() == -1) {
			payload.put("y", "Y-PLACEHOLDER");
		} else {
			payload.put("y", position.getY());
		}
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		return new EquipletStepMessage(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
	}

	private EquipletStepMessage moveZ(BasicDBObject parameters) {
		Position position = new Position((BasicDBObject) parameters.get("Position"));
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}
		BasicDBObject payload = new BasicDBObject();
		if (position.getZ() == -1) {
			payload.put("z", "Z-PLACEHOLDER");
		} else {
			payload.put("z", position.getZ());
		}
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		return new EquipletStepMessage(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
	}

}
