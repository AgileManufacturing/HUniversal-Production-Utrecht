/**
 * @file src/REXOS/MAS/HAL/capabilities/PickAndPlace.java
 * @brief Translate product steps to hardware steps.
 * @date Created: 2014-03-21
 * 
 * @author Aristides Ayala Mendoza
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

import generic.ProductStep;

import java.util.ArrayList;

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

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

/**
 * Pick and place capability class that translate
 * 
 *
 */
public class PickAndPlace extends Capability {
	public final static String SERVICE_IDENTIFIER = "place";
	
	/**
	 * 
	 * @param moduleFactory
	 */
	public PickAndPlace(ModuleFactory moduleFactory) {
		super(moduleFactory, "PickAndPlace");
	}

	/**
	 * @throws ModuleTranslatingException 
	 * @see Capability#translateProductStep(ProductStep)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws CapabilityException {
		JsonObject target = productStep.getCriteria().get(ProductStep.TARGET).getAsJsonObject();
		JsonArray subjects = productStep.getCriteria().get(ProductStep.SUBJECTS).getAsJsonArray();
		
		if(productStep.getService().getName().equals(SERVICE_IDENTIFIER) == false) {
			String message = "Recieved a service (" + productStep.getService() + "which is not supported by this capability.";
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
			throw new IllegalArgumentException(message);	
		}
		if(target == null) {
			String message = "Recieved a illegaly formatted product step: " + productStep;
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
			throw new IllegalArgumentException(message);
		}
		if(subjects.size() == 0) {
			String message = "Recieved a product step which has no subjects: " + productStep;
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
			throw new IllegalArgumentException(message);
		}
		
		
		// pick
		// We assume that the first subject is the subject to pick
		JsonObject subject = subjects.getAsJsonArray().get(0).getAsJsonObject();
		JsonObject subjectMoveCommand = subject.get("move").getAsJsonObject();
		
		JsonObject pickCommand = new JsonObject();
		pickCommand.add("pick", null);
		pickCommand.add("move", subjectMoveCommand);
		
		JsonObject pickOriginPlacementParameters = new JsonObject();
		pickOriginPlacementParameters.addProperty("identifier", subject.get(CompositeStep.IDENTIFIER).getAsString());
		OriginPlacement pickOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_IDENTIFIER, pickOriginPlacementParameters);
		
		CompositeStep pick = new CompositeStep(productStep, pickCommand, pickOriginPlacement);
		Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "pick: " + pick);
		
		// place
		JsonObject targetMoveCommand = target.get("move").getAsJsonObject();
		
		JsonObject placeCommand = new JsonObject();
		placeCommand.add("place", null);
		placeCommand.add("move", targetMoveCommand);
		
		JsonObject placeOriginPlacementParameters = new JsonObject();
		placeOriginPlacementParameters.addProperty("identifier", target.get(CompositeStep.IDENTIFIER).getAsString());
		OriginPlacement placeOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_IDENTIFIER, placeOriginPlacementParameters);
		
		CompositeStep place = new CompositeStep(productStep, placeCommand, placeOriginPlacement);
		Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "place: " + place);
		
		// Translate to hardwareSteps
		ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
		ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
		compositeSteps.add(pick);
		compositeSteps.add(place);
			
		ArrayList<HardwareStep> hardwareSteps = translateCompositeStep(modules, compositeSteps);
		Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
		return hardwareSteps;
	}
}