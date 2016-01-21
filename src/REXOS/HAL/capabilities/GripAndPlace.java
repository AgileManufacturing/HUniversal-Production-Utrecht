/**
 * @file src/REXOS/HAL/capabilities/GripAndPlace.java
 * @brief Translate product steps to hardware steps.
 * @date Created: 2015-11-21
 * 
 * @author Aristides Ayala Mendoza
 * @author Casper Wolf
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2014, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
 
package HAL.capabilities;
import generic.Criteria;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Capability;
import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.OriginPlacement;
import HAL.steps.OriginPlacementType;


public class GripAndPlace extends Capability {
	public final static String SERVICE_IDENTIFIER = "place"; //not sure yet
	private final String CHECK_CRITERIA = "check";

	public GripAndPlace(ModuleFactory moduleFactory) {
		super(moduleFactory, "GripAndPlace");
	}

	public ArrayList<HardwareStep> translateProductStep(String service, JSONObject criteria) throws CapabilityException {
		try {
			JSONObject target = criteria.getJSONObject(Criteria.TARGET);
			JSONArray subjects = criteria.getJSONArray(Criteria.SUBJECTS);
			JSONObject check = criteria.getJSONObject(CHECK_CRITERIA); // official solution?? If so, add to generic.Criteria.java

			if (service.equals(SERVICE_IDENTIFIER) == false) {
				String message = "Received a service (" + service + "which is not supported by this capability.";
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
				throw new IllegalArgumentException(message);
			}
			if (subjects.length() == 0) {
				String message = "Received a job which has no subjects: " + service + " with criteria " + criteria;
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
				throw new IllegalArgumentException(message);
			}
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "IS THERE ANYBODY OUT THERE?");
			// We assume that the first subject is the subject to pick
			JSONObject subject = subjects.getJSONObject(0);
			ArrayList<CompositeStep> dynamicCompositeSteps = generateStepFromOffset(check,subject); // NULL unless the rotation is wrong, in which case it returns the steps as arraylist 
			// pick
			JSONObject subjectMoveCommand = subject.getJSONObject("move");
			JSONObject subjectRotateCommand = subject.getJSONObject("rotate");

			JSONObject pickCommand = new JSONObject();
			pickCommand.put("pick", JSONObject.NULL);
			pickCommand.put("move", subjectMoveCommand);
			pickCommand.put("rotate", subjectRotateCommand);

			JSONObject pickOriginPlacementParameters = new JSONObject();
			pickOriginPlacementParameters.put("identifier", subject.getString(CompositeStep.IDENTIFIER));
			OriginPlacement pickOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, pickOriginPlacementParameters);

			CompositeStep pick = new CompositeStep(service, pickCommand, pickOriginPlacement);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "pick: " + pick);

			// place
			JSONObject targetMoveCommand = target.getJSONObject("move");
			JSONObject targetRotateCommand = target.getJSONObject("rotate");

			JSONObject placeCommand = new JSONObject();
			placeCommand.put("place", JSONObject.NULL);
			placeCommand.put("move", targetMoveCommand);
			placeCommand.put("rotate", targetRotateCommand);

			JSONObject placeOriginPlacementParameters = new JSONObject();
			placeOriginPlacementParameters.put("identifier", target.getString(CompositeStep.IDENTIFIER));
			OriginPlacement placeOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, placeOriginPlacementParameters);

			CompositeStep place = new CompositeStep(service, placeCommand, placeOriginPlacement);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "place: " + place);

			// Translate to hardwareSteps
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
			ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
			compositeSteps.add(pick);
			compositeSteps.add(place);
			ArrayList<HardwareStep> hardwareSteps;
			if (dynamicCompositeSteps != null){ // add in the dynamic extra steps with the normal procedure steps
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "Executing extra step!");
				hardwareSteps = translateCompositeStep(modules, dynamicCompositeSteps);
				ArrayList<HardwareStep> hardwareSteps2 = translateCompositeStep(modules, compositeSteps);
				for (HardwareStep step : hardwareSteps2){
					hardwareSteps.add(step);
				}
			} else {
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "No extra step");
				hardwareSteps = translateCompositeStep(modules, compositeSteps);
			}
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
			return hardwareSteps;
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, "Unable to translate due to illegally formatted JSON", ex);
			throw new CapabilityException("Unable to translate due to illegally formatted JSON", ex);
		}
	}

	private final double ALLOWED_ROTATION_DIFFERENCE = 0.1;
	private ArrayList<CompositeStep> generateStepFromOffset(JSONObject subjectCheck,JSONObject subjectLocation) throws JSONException {
		double desiredRotation = subjectCheck.getDouble("desiredRotation");
		double detectedRotation = subjectCheck.getDouble("detectedRotation");
		if (Math.abs(desiredRotation - detectedRotation) > ALLOWED_ROTATION_DIFFERENCE){
			//correctional action has to be taken
			double requiredRotation = desiredRotation - detectedRotation;
			// create a step to pick at detectedRotation and place at desiredRotation
			// pick
			JSONObject pickCommand = new JSONObject();
			pickCommand.put("pick", JSONObject.NULL);
			pickCommand.put("move",subjectLocation.getJSONObject("move"));
			JSONObject rotateCommand = new JSONObject();
			rotateCommand.put("x", 0.0);
			rotateCommand.put("y", 0.0);
			rotateCommand.put("z", 0.0);
			JSONObject rotateApproach = new JSONObject();
			rotateApproach.put("x", 0.0);
			rotateApproach.put("y", 0.0);
			rotateApproach.put("z", 0.0);
			pickCommand.put("rotate", rotateCommand);

			JSONObject pickOriginPlacementParameters = new JSONObject();
			pickOriginPlacementParameters.put("identifier", subjectLocation.getString(CompositeStep.IDENTIFIER));
			OriginPlacement pickOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, pickOriginPlacementParameters);

			CompositeStep pick = new CompositeStep(SERVICE_IDENTIFIER, pickCommand, pickOriginPlacement);

			JSONObject placeCommand = new JSONObject();
			placeCommand.put("place", JSONObject.NULL);
			placeCommand.put("move", subjectLocation.getJSONObject("move"));
			JSONObject rotateCommand2 = new JSONObject();
			rotateCommand2.put("x", 0.0);
			rotateCommand2.put("y", 0.0);
			rotateCommand2.put("z", requiredRotation);
			placeCommand.put("rotate", JSONObject.NULL);

			CompositeStep place = new CompositeStep(SERVICE_IDENTIFIER,placeCommand, pickOriginPlacement);

			ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
			compositeSteps.add(pick);
			compositeSteps.add(place);

			//ArrayList<HardwareStep> hardwareSteps = translateCompositeStep(modules, compositeSteps);

			return compositeSteps; // returns the extra compositeSteps to perform the dynamic step (not translated here to avoid double module lookup)
		} else {
			return null; // no extra step required
		}
	}
}


/*

PERSLUCHT

DOCUMENT autosuspend for USB for camera node

grip and place action with dynamic step implementation:

pick
identifier
offset 
rotation

place 
identifier
offset
rotation

check
desiredRotation
rotation
desiredCorrection -> generated from a pick and place action, pick at rot place back at desiredRot



 */

