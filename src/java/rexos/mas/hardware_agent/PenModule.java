/**
 * @file rexos/mas/hardware_agent/PenModule.java
 * @brief Provides the data to be used for hardware agents.
 * @date Created: 12-04-13
 * 
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

import com.mongodb.BasicDBObject;

/**
 * Module class that contains the functions for the pen module.
 */
public class PenModule extends Module {
	/**
	 * @var int PEN_OFFSET
	 *      A static value that contains the offset of the pen in relation to the movement module.
	 */
	private static final int PEN_OFFSET = 0;

	/**
	 * @var double PEN_SIZE
	 *      A static value that contains the size of the pen in centimeters.
	 */
	private static final double PEN_SIZE = 1.0;

	/**
	 * @var Module movementModule
	 *      The module that moves this module.
	 */
	private Module movementModule;
	
	/**
	 * @see Module#getEquipletSteps(int, BasicDBObject)
	 */
	@Override
	public EquipletStep[] getEquipletSteps(int stepType, BasicDBObject parameters) {
		EquipletStep[] equipletSteps;
		ArrayList<EquipletStep> steps;

		// gets the newest code for the movementModule.
		int movementModuleId = findMovementModule(getConfiguration());
		movementModule = getModuleFactory().getModuleById(movementModuleId);

		// switch to determine which steps to make.
		switch(stepType) {
			case 3: // case for the equiplet function draw line.
				steps = new ArrayList<EquipletStep>();

				// only get steps when start and end position are available.
				if(parameters.containsField("startPosition") && parameters.containsField("endPosition")) {
					// get start position
					Position startPosition = new Position((BasicDBObject) parameters.get("startPosition"));
					// get end postion
					Position endPosition = new Position((BasicDBObject) parameters.get("endPosition"));
					// get the delta of the x and the y
					double deltaX = endPosition.getX() - startPosition.getX();
					double deltaY = endPosition.getY() - startPosition.getY();
					// get the length of the line.
					double lineLength = Math.hypot(deltaX, deltaY);

					// get the number of steps to perform and the x and y delta for each step.
					int numberOfSteps = Math.max(1, (int) (lineLength / PEN_SIZE));
					Position step = new Position(deltaX / numberOfSteps, deltaY / numberOfSteps);

					// loop over all the steps and create them.
					for(int i = 0; i <= numberOfSteps; i++) {
						steps.addAll(Arrays.asList(getStepsForDot(new BasicDBObject("position", startPosition
								.toBasicDBObject()))));
						startPosition.translate(step);
					}// get the position of the dot.
					Position position = new Position();
					position.setZ(0.0);

					// create the new parameters
					BasicDBObject saveParameters = new BasicDBObject("position", position.toBasicDBObject());
					saveParameters.put("extraSize", PEN_OFFSET);
					steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, saveParameters)));
				}

				// convert the ArrayList to an array and return it.
				equipletSteps = new EquipletStep[steps.size()];
				return steps.toArray(equipletSteps);
			case 4: // case for the equiplet function put dot.
				return getStepsForDot(parameters);
			default:
				break;
		}
		// if this module can't handle the stepType return no steps.
		return new EquipletStep[0];
	}

	/**
	 * @see Module#fillPlaceHolders(EquipletStep[], BasicDBObject)
	 */
	@Override
	public EquipletStep[] fillPlaceHolders(EquipletStep[] steps, BasicDBObject parameters) {
		// return the filled in steps.
		return steps;
	}

	/**
	 * @see Module#isLeadingForServices()
	 */
	@Override
	public int[] isLeadingForServices() {
		return new int[] { 2 };
	}

	/**
	 * Function to find the movementModule of this module in the configuration HashMap.
	 * 
	 * @param hashMap A hashMap with the configuration of the module.
	 * @return The int id of the movementModule.
	 */
	private int findMovementModule(HashMap<Integer, Object> hashMap) {
		// check if the id is in this layer, return there is no movementModule(-1).
		if(hashMap.containsKey(getId())) {
			return -1;
		}

		for(int key : hashMap.keySet()) {
			try {
				// check each value of the HashMap to see if it is a HashMap
				HashMap<Integer, Object> tempHashMap = (HashMap<Integer, Object>) hashMap.get(key);
				// check if the new HashMap contains the key of
				// this module then return the key of the HashMap(this is the movementModule).
				if(tempHashMap.containsKey(getId())) {
					return key;
				}
				// if the key isn't found search recursive in the next HashMap.
				int tempId = findMovementModule(tempHashMap);
				// if the next HashMap has found the module return it
				if(tempId != -1) {
					return tempId;
				}
			} catch(ClassCastException e) {/* its no HashMap so do nothing */
			}
		}
		return -1;
	}

	/**
	 * Function for getting the steps for placing a dot.
	 * 
	 * @param parameters The parameters used for placing the dot.
	 * @return An array of the generated equipletSteps
	 */
	private EquipletStep[] getStepsForDot(BasicDBObject parameters) {
		ArrayList<EquipletStep> steps = new ArrayList<EquipletStep>();

		// get the position of the dot.
		Position position = new Position((BasicDBObject) parameters.get("position"));
		position.setZ(0.0);

		// create the new parameters
		BasicDBObject dotParameters = new BasicDBObject("position", position.toBasicDBObject());
		dotParameters.put("extraSize", PEN_OFFSET);

		// get steps from the movementModule to move to the safe movement plane.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, dotParameters)));

		// get steps from the movementModule to move on the x and y axis.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(2, dotParameters)));

		// get steps from the movementModule to move on the z axis.
		steps.addAll(Arrays.asList(movementModule.getEquipletSteps(3, dotParameters)));

		// get steps from the movementModule to move to the safe movement plane(not needed).
		//steps.addAll(Arrays.asList(movementModule.getEquipletSteps(1, dotParameters)));

		// convert the ArrayList to an array and return it.
		EquipletStep[] equipletSteps = new EquipletStep[steps.size()];
		return steps.toArray(equipletSteps);
	}
}
