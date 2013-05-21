/**
 * @file rexos/mas/hardware_agent/DeltaRobotModule.java
 * @brief Provides a deltaRobotModule.
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

import rexos.mas.data.Position;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

/**
 * Module class that contains the functions for the deltarobot module.
 */
public class DeltaRobotModule extends Module {
	/**
	 * @var double SAFE_MOVEMENT_PLANE
	 * A static value that contains the height of the safe movement plane.
	 */
	private static final double SAFE_MOVEMENT_PLANE = 6;

	/**
	 * Empty Constructory
	 */
	public DeltaRobotModule() {}

	/**
	 * @see Module#getEquipletSteps(int, BasicDBObject)
	 */
	@Override
	public EquipletStep[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		//switch to determine which steps to make.
		switch (stepType) {
		case 1: //case to move to the safe movement plane.
			//returns the steps for moving to the safe movement plane.
			return new EquipletStep[] { moveToSafePlane(parameters) };
		case 2: //case to move on the x and y axis.
			//returns the steps for the movement on the x and y axis.
			return new EquipletStep[] { moveXY(parameters) };
		case 3: //case to move on the z axis
			//returns the steps for the movement on the z axis.
			return new EquipletStep[] { moveZ(parameters) };
		default:
			break;
		}
		//if this module can't handle the stepType return no steps.
		return new EquipletStep[0];
	}

	/**
	 * @see Module#fillPlaceHolders(EquipletStep[], BasicDBObject)
	 */
	@Override
	public EquipletStep[] fillPlaceHolders(EquipletStep[] steps, BasicDBObject parameters) {
		//get the new position parameters from the parameters
		Position position = new Position((BasicDBObject) parameters.get("position"));
		
		//loop over the steps.
		for (EquipletStep step : steps) {
			//get the lookUpParameters and the payload and
			//replace the placeholders with real data.
			InstructionData instructionData = step.getInstructionData();
			BasicDBObject lookUpParameters = instructionData.getLookUpParameters();
			BasicDBObject payload = instructionData.getPayload();
			if (lookUpParameters.getString("ID").equals("RELATIVE-TO-PLACEHOLDER")) {
				lookUpParameters.put("ID", position.getRelativeToPart());
			}
			if (payload.containsField("x") && payload.getString("x").equals("X-PLACEHOLDER")) {
				payload.put("x", position.getX());
			}
			if (payload.containsField("y") && payload.getString("y").equals("Y-PLACEHOLDER")) {
				payload.put("y", position.getY());
			}
			if (payload.containsField("z") && payload.getString("z").equals("Z-PLACEHOLDER")) {
				payload.put("z", position.getZ());
			}
		}
		//returns the filled in steps.
		return steps;
	}

	/**
	 * @see Module#isLeadingForSteps()
	 */
	@Override
	public int[] isLeadingForSteps() {
		return new int[] {};
	}

	/**
	 * Function that builds the step for moving to the safe plane.
	 * @param parameters The parameters to use by this step.
	 * @return EquipletStep to move to the safe plane.
	 */
	private EquipletStep moveToSafePlane(BasicDBObject parameters) {
		//get the extraSize from the parameters(e.g. Size of the module on this module)
		double extraSize = parameters.getDouble("extraSize");

		//create the lookUpParameters
		BasicDBObject lookUpParameters = new BasicDBObject("ID", ((BasicDBObject) parameters.get("position")).get("relativeToPart"));
		//create the payload
		BasicDBObject payload = new BasicDBObject("z", extraSize + SAFE_MOVEMENT_PLANE);
		//create the instruction data
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		//create an EquipletStep and return it.
		EquipletStep step = new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
		return step;
	}

	/**
	 * Function that builds the step for moving on the x and y axis.
	 * @param parameters The parameters to use by this step. 
	 * @return EquipletStep to move on the x and y axis.
	 */
	private EquipletStep moveXY(BasicDBObject parameters) {
		//get the position parameters from the parameters.
		Position position = new Position((BasicDBObject) parameters.get("position"));
		
		//fill in the lookUpParameters
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}
		
		//fill in the payload parameters
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
		//create the instruction data.
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		//create the EquipletStep and return it.
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
	}

	/**
	 * Function that builds the step for moving on the z axis.
	 * @param parameters The parameters to use by this step.
	 * @return EquipletStep to move on the z axis.
	 */
	private EquipletStep moveZ(BasicDBObject parameters) {
		//get the position parameters from the parameters
		Position position = new Position((BasicDBObject) parameters.get("position"));
		
		//fill in the lookUpParameters.
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart());
		}
		
		//fill in the payload parameters.
		BasicDBObject payload = new BasicDBObject();
		if (position.getZ() == -1) {
			payload.put("z", "Z-PLACEHOLDER");
		} else {
			payload.put("z", position.getZ());
		}
		
		//create the instruction data.
		InstructionData instructionData = new InstructionData("move", "deltarobot", "FIND_ID", lookUpParameters, payload);
		//create the EquipletStep and return it. 
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new TimeData(4));
	}

}
