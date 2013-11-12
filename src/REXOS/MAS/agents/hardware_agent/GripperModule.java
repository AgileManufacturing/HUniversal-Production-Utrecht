/**
 * @file rexos/mas/hardware_agent/GripperModule.java
 * @brief Provides the data to be used for hardware agents.
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 * @author Duncan Jenkins
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

package agents.hardware_agent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.Part;
import agents.data_classes.Position;
import agents.data_classes.StepStatusCode;

import com.mongodb.BasicDBObject;

/**
 * Module class that contains the functions for the gripper module.
 */
public class GripperModule extends Module {
	/**
	 * @var int GRIPPER_SIZE
	 * A static value that contains the size of the gripper.
	 */
	private static final double GRIPPER_SIZE = 18.24;

	/**
	 * @var Module movementModule
	 * The module that moves this module.
	 */
	private Module movementModule;

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
		
		steps = new ArrayList<EquipletStep>();
		
		double crateHeight = 0;
		if(parameters.containsField("height")){
			crateHeight = parameters.getDouble("height");
		}

		//get the parameters and put extra values in it.
		BasicDBObject moveParameters = new BasicDBObject();
		moveParameters.put("extraSize", GRIPPER_SIZE + crateHeight);
		moveParameters.put("position", new Position().toBasicDBObject());

		//get steps from the movementModule to move to the safe movement plane.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));
		//get steps from the movementModule to move on the x and y axis relative to a crate.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, moveParameters)));
		//get steps from the movementModule to move on the z axis.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, moveParameters)));

		//switch to determine which steps to make.
		switch (stepType) {
		case 1: //case for the equiplet function pick.
			//get step for activating the gripper.
			steps.add(activateGripper(moveParameters));
			break;
		case 2: //case for the equiplet function place. 
			//get step for deactivating the gripper.
			steps.add(deactivateGripper(moveParameters));
			break;
		default:
			//if this module can't handle the stepType return no steps. 
			return new EquipletStep[0];
		}
		
		//get steps from the movementModule to move to the safe movement plane.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, moveParameters)));

		//convert the ArrayList to an array and return it.
		equipletSteps = new EquipletStep[steps.size()];
		return steps.toArray(equipletSteps);
	}

	/**
	 * @see Module#fillPlaceHolders(EquipletStep[], BasicDBObject)
	 */
	@Override
	public EquipletStep[] fillPlaceHolders(EquipletStep[] steps, BasicDBObject parameters) {
		//Logger.log(LogLevel.DEBUG, "parameters: " + parameters.keySet());
		
		//You have your crateID from the DBObject parameters.
		//Get the cratePart from the knowledgeDB -- maybe store crate object instead of string.
		//actually we have to get the specific crate dimensions etc. WE CAN HARDCODE THIS FOR THE MOMENT!
		//translate the row/col to X,Y,Z ACCORDING to the crate dimensions.
		double crateHeight = 0;
		if(parameters.containsField("height")){
			crateHeight = parameters.getDouble("height");
		}
		
		double crateDimension = 46; // 46mm x 46mm
        double crateSlots = 4;
        double crateSlotDimension = 11.5;
        double crateSlotMidPoint = 5.75;
        double extraSize = GRIPPER_SIZE + crateHeight;
		
		Part part = new Part((BasicDBObject)parameters.get("crate"));
		Position position = new Position((parameters.getDouble("row") * crateSlotDimension + crateSlotMidPoint) - 23, (parameters.getDouble("column") * crateSlotDimension + crateSlotMidPoint) - 23, 5.0, part);
		
		//get the newest code of the movementModule.
		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		//make an ArrayList containing all the steps of the movementModule.
		ArrayList<EquipletStep> movementModuleSteps = new ArrayList<EquipletStep>();
		//loop over all the given EquipletSteps.
		for (EquipletStep step : steps) {
			InstructionData instructionData = step.getInstructionData();
			BasicDBObject lookUpParameters = instructionData.getLookUpParameters();
			BasicDBObject payload = instructionData.getPayload();
			
			if(lookUpParameters.containsField("ID")) {
				if(lookUpParameters.getString("ID").equals("RELATIVE-TO-PLACEHOLDER") && position.getRelativeToPart() != null) {
					lookUpParameters.put("ID", position.getRelativeToPart().getPartName());
				}
			}

			if(payload.containsField("x") && payload.getString("x").equals("X-PLACEHOLDER")) {
				payload.put("x", position.getX());
			}
			
			if(payload.containsField("y") && payload.getString("y").equals("Y-PLACEHOLDER")) {
				payload.put("y", position.getY());
			}
			
			if(payload.containsField("z") && payload.getString("z").equals("Z-PLACEHOLDER")) {
				if(position.getZ() == null)	{
					payload.put("z", 0 + extraSize);
				} else {
					payload.put("z", position.getZ() + extraSize);
				}
			}
			
			//If the step is made by the movementModule add it to the ArrayList.
			if (step.getModuleId() == movementModule.getId()) {
				movementModuleSteps.add(step);
			}
		}
		//let the movementModule fill in his steps.
		//EquipletStep[] temp = new EquipletStep[movementModuleSteps.size()];
		//movementModule.fillPlaceHolders(movementModuleSteps.toArray(temp), newParameters);
		//return the filled in steps.
		EquipletStep[] temp = new EquipletStep[movementModuleSteps.size()];
		return movementModuleSteps.toArray(temp);
	}

	/**
	 * @see Module#isLeadingForServices()
	 */
	@Override
	public int[] isLeadingForServices() {
		return new int[] { 1 };
	}

	/**
	 * Function to find the movementModule of this module in the configuration HashMap.
	 * @param configuration A hashMap with the configuration of the module.
	 * @return The int id of the movementModule.
	 */
	@SuppressWarnings({
			"rawtypes", "cast"
	})
	private int findMovementModule(HashMap<Integer, HashMap> configuration) {
		//check if the id is in this layer, return there is no movementModule(-1).
		if (configuration.containsKey(getId())) {
			return -1;
		}
		
		for (int key : configuration.keySet()) {
			//check each value of the HashMap to see if it is a HashMap
			HashMap<Integer, HashMap> subConfiguration = (HashMap<Integer, HashMap>) configuration.get(key);
			//check if the new HashMap contains the key of 
			//this module then return the key of the HashMap(this is the movementModule).
			if (subConfiguration.containsKey(getId())) {
				return key;
			}
			//if the key isn't found search recursive in the next HashMap.
			int tempId = findMovementModule(subConfiguration);
			//if the next HashMap has found the module return it
			if (tempId != -1) {
				return tempId;
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
		BasicDBObject lookUpParameters = new BasicDBObject();
		
		//create the instruction data.
		InstructionData instructionData = new InstructionData("activate", "gripper", "NULL", lookUpParameters, new BasicDBObject());
		//create and return the step.
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new BasicDBObject(), new TimeData(1));
	}

	/**
	 * Function that builds the step for deactivating the gripper.
	 * @param parameters The parameters to use by this step.
	 * @return EquipletStep to deactivate the gripper.
	 */
	private EquipletStep deactivateGripper(BasicDBObject parameters) {
		BasicDBObject lookUpParameters = new BasicDBObject();
		
		//create instruction data.
		InstructionData instructionData = new InstructionData("deactivate", "gripper", "NULL", lookUpParameters, new BasicDBObject());
		//create and return the step.
		return new EquipletStep(null, getId(), instructionData, StepStatusCode.EVALUATING, new BasicDBObject(), new TimeData(1));
	}

}
