/**
 * @file src/REXOS/MAS/HAL/capabilities/PickAndPlace.java
 * @brief Translate product steps to hardware steps.
 * @date Created: 2016-06-10
 * 
 * @author Feiko Wielsma
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

/**
 * Pick and place capability class that translate with rotation.
 * 
 *
 */
public class BreakWithRotation extends Capability {
	public final static String SERVICE_IDENTIFIER = "break";

	/**
	 * 
	 * @param moduleFactory
	 */
	public BreakWithRotation(ModuleFactory moduleFactory) {
		super(moduleFactory, "BreakWithRotation");
	}

	/**
	 * @throws ModuleTranslatingException
	 * @see Capability#translateProductStep(String, JSONObject)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(String service, JSONObject criteria) throws CapabilityException {
		try {
			JSONObject target = criteria.getJSONObject(Criteria.TARGET);
			JSONArray subjects = criteria.getJSONArray(Criteria.SUBJECTS);

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

			// pick
			// We assume that the first subject is the subject to pick
			JSONObject subject = subjects.getJSONObject(0);
			JSONObject subjectMoveCommand = subject.getJSONObject("move");
			JSONObject subjectRotateCommand = subject.getJSONObject("rotate");

			JSONObject subject2 = subjects.getJSONObject(1);
			JSONObject subject2MoveCommand = subject2.getJSONObject("move");
			JSONObject subject2RotateCommand = subject2.getJSONObject("rotate");

			JSONObject pickCommand = new JSONObject();
			pickCommand.put("pick", JSONObject.NULL);
			pickCommand.put("move", subjectMoveCommand);
			pickCommand.put("rotate", subjectRotateCommand);

			JSONObject pickOriginPlacementParameters = new JSONObject();
			pickOriginPlacementParameters.put("identifier", subject.getString(CompositeStep.IDENTIFIER));
			OriginPlacement pickOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, pickOriginPlacementParameters);

			CompositeStep pick = new CompositeStep(service, pickCommand, pickOriginPlacement);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "pick: " + pick);

			// break
			JSONObject breakCommand = new JSONObject();
			//breakCommand.put("place", JSONObject.NULL);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "break");
			breakCommand.put("move", subject2MoveCommand);
			breakCommand.put("rotate", subject2RotateCommand);

			JSONObject breakOriginPlacementParameters = new JSONObject();
			breakOriginPlacementParameters.put("identifier", subject2.getString(CompositeStep.IDENTIFIER));
			OriginPlacement breakOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, breakOriginPlacementParameters);

			CompositeStep breakStep = new CompositeStep(service, breakCommand, breakOriginPlacement);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "break: " + breakStep);

			// place
			JSONObject targetMoveCommand = target.getJSONObject("move");
			JSONObject targetRotateCommand = target.getJSONObject("rotate");

			JSONObject placeCommand = new JSONObject();
			placeCommand.put("place", JSONObject.NULL);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "place");
			placeCommand.put("move", targetMoveCommand);
			placeCommand.put("rotate", targetRotateCommand);

			JSONObject placeOriginPlacementParameters = new JSONObject();
			placeOriginPlacementParameters.put("identifier", target.getString(CompositeStep.IDENTIFIER));
			OriginPlacement placeOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_PART_ORIGIN, placeOriginPlacementParameters);

			CompositeStep placeStep = new CompositeStep(service, placeCommand, placeOriginPlacement);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "place: " + placeStep);
			
			// Translate to hardwareSteps
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
			ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
			compositeSteps.add(pick);
			compositeSteps.add(breakStep);
			compositeSteps.add(placeStep);


			ArrayList<HardwareStep> hardwareSteps = translateCompositeStep(modules, compositeSteps);
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
			return hardwareSteps;
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, "Unable to translate due to illegally formatted JSON", ex);
			throw new CapabilityException("Unable to translate due to illegally formatted JSON", ex);
		}
	}
}
