/**
 * @file rexos/mas/hardware_agent/GripperModule.java
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

/**
 * Module class that contains the functions for the gripper module.
 */
public class GripperModule extends Module {
	/**
	 * @var int GRIPPER_SIZE
	 * A static value that contains the size of the gripper.
	 */
	private static final int GRIPPER_SIZE = 2;

	/**
	 * @var Module movementModule
	 * The module that moves this module.
	 */
	private Module movementModule;

	/**
	 * Empty Constructor
	 */
	public GripperModule() {}

	/**
	 * @see Module#getEquipletSteps(int, BasicDBObject)
	 */
	@Override
	public EquipletStep[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		EquipletStep[] equipletSteps;
		ArrayList<EquipletStep> steps;

		//gets the newest code for the movementModule.
		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		//switch to determine which steps to make.
		switch (stepType) {
		case 1: //case for the equiplet function pick.
			steps = new ArrayList<EquipletStep>();

			//get the parameters and put extra values in it.
			BasicDBObject moveParameters = (BasicDBObject) parameters.copy();
			moveParameters.put("extraSize", GRIPPER_SIZE);

			//get steps from the movementModule to move to the safe movement plane.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));
			//get steps from the movementModule to move on the x and y axis.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, moveParameters)));
			//get steps from the movementModule to move on the z axis.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, moveParameters)));
			//get step for activating the gripper.
			steps.add(activateGripper(moveParameters));
			//get steps from the movementModule to move to the safe movement plane.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));

			//convert the ArrayList to an array and return it.
			equipletSteps = new EquipletStep[steps.size()];
			return steps.toArray(equipletSteps);
		case 2: //case for the equiplet function place. 
			steps = new ArrayList<EquipletStep>();

			//get the parameters and put extra values in it.
			moveParameters = (BasicDBObject) parameters.copy();
			moveParameters.put("extraSize", GRIPPER_SIZE);

			//get steps from the movementModule to move to the safe movement plane.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));
			//get steps from the movementModule to move on the x and y axis.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, moveParameters)));
			//get steps from the movementModule to move on the z axis.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, moveParameters)));
			//get step for deactivating the gripper.
			steps.add(deactivateGripper(moveParameters));
			//get steps from the movementModule to move to the safe movement plane.
			steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));

			//convert the ArrayList to an array and return it.
			equipletSteps = new EquipletStep[steps.size()];
			return steps.toArray(equipletSteps);
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

		//get the newest code of the movementModule.
		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		//make an ArrayList containing all the steps of the movementModule.
		ArrayList<EquipletStep> movementModuleSteps = new ArrayList<EquipletStep>();
		//loop over all the given EquipletSteps.
		for (EquipletStep step : steps) {
			//If the step is made by the movementModule add it to the ArrayList.
			if (step.getModuleId() == movementModule.getId()){
				movementModuleSteps.add(step);
			}else {
				//fill the placeholder of the look_up_parameters if needed.
				InstructionData instructionData = step.getInstructionData();
				BasicDBObject lookUpParameters = instructionData.getLookUpParameters();
				if (lookUpParameters.getString("ID").equals("RELATIVE-TO-PLACEHOLDER")) {
					lookUpParameters.put("ID", position.getRelativeToPart().getId());
				}
			}
		}
		//let the movementModule fill in his steps.
		EquipletStep[] temp = new EquipletStep[movementModuleSteps.size()];
		movementModule.fillPlaceHolders(movementModuleSteps.toArray(temp), parameters);
		//return the filled in steps.
		return steps;
	}

	/**
	 * @see Module#isLeadingForSteps()
	 */
	@Override
	public int[] isLeadingForSteps() {
		return new int[] { 1, 2 };
	}

	/**
	 * Function to find the movementModule of this module in the configuration HashMap.
	 * @param hashMap A hashMap with the configuration of the module.
	 * @return The int id of the movementModule.
	 */
	private int findMovementModule(HashMap<Integer, Object> hashMap) {
		//check if the id is in this layer, return there is no movementModule(-1).
		if (hashMap.containsKey(getId())) {
			return -1;
		}
		
		for (int key : hashMap.keySet()) {
			try {
				//check each value of the HashMap to see if it is a HashMap
				HashMap<Integer, Object> tempHashMap = (HashMap<Integer, Object>) hashMap.get(key);
				//check if the new HashMap contains the key of 
				//this module then return the key of the HashMap(this is the movementModule).
				if (tempHashMap.containsKey(getId())) {
					return key;
				}
				//if the key isn't found search recursive in the next HashMap.
				int tempId = findMovementModule(tempHashMap);
				//if the next HashMap has found the module return it
				if (tempId != -1) {
					return tempId;
				}
			} catch (ClassCastException e) {/* its no HashMap so do nothing */
			}
		}
		return -1;
	}

	/**
	 * Function that builds the step for activating the gripper.
	 * @param parameters The parameters to use by this step.
	 * @return EquipletStep to activate the gripper.
	 */
	private EquipletStep activateGripper(BasicDBObject parameters) {
		//get the position parameters from the parameters.
		Position position = new Position((BasicDBObject) parameters.get("position"));
		
		//fill int het lookUpParameters with values or placeholders
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == null || position.getRelativeToPart().getId() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart().getId());
		}
		
		//create the instruction data.
		InstructionData instructionData = new InstructionData("activate", "gripper", "FIND_ID", lookUpParameters, new BasicDBObject());
		//create and return the step.
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new BasicDBObject(), new TimeData(1));
	}

	/**
	 * Function that builds the step for deactivating the gripper.
	 * @param parameters The parameters to use by this step.
	 * @return EquipletStep to deactivate the gripper.
	 */
	private EquipletStep deactivateGripper(BasicDBObject parameters) {
		//get the position parameters from the parameters.
		Position position = new Position((BasicDBObject) parameters.get("position"));
		
		//fill in the lookUpParameters with values or placeholders.
		BasicDBObject lookUpParameters = new BasicDBObject();
		if (position.getRelativeToPart() == null || position.getRelativeToPart().getId() == -1) {
			lookUpParameters.put("ID", "RELATIVE-TO-PLACEHOLDER");
		} else {
			lookUpParameters.put("ID", position.getRelativeToPart().getId());
		}

		//create instruction data.
		InstructionData instructionData = new InstructionData("deactivate", "gripper", "FIND_ID", lookUpParameters, new BasicDBObject());
		//create and return the step.
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new BasicDBObject(), new TimeData(1));
	}

}
