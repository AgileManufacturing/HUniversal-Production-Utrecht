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

import libraries.log.LogLevel;
import libraries.log.LogSection;
import libraries.log.Logger;
import HAL.Capability;
import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

/**
 * Pick and place capability class that translate with rotation.
 * 
 *
 */
public class PickAndPlaceWithRotation extends Capability {
	public final static String SERVICE_IDENTIFIER = "place";
	
	/**
	 * 
	 * @param moduleFactory
	 */
	public PickAndPlaceWithRotation(ModuleFactory moduleFactory) {
		super(moduleFactory, "PickAndPlaceWithRotation");
	}

	/**
	 * @throws ModuleTranslatingException 
	 * @see Capability#translateProductStep(ProductStep)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws CapabilityException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		JsonObject target = productStep.getCriteria().get(ProductStep.TARGET).getAsJsonObject();
		JsonArray subjects = productStep.getCriteria().get(ProductStep.SUBJECTS).getAsJsonArray();
		
		if(productStep.getService().getName().equals(SERVICE_IDENTIFIER) && subjects != null && target != null){	
			for(int i = 0; i < subjects.getAsJsonArray().size(); i++){
				//Pick
				JsonObject subjectMoveCommand = subjects.getAsJsonArray().get(i).getAsJsonObject().get("move").getAsJsonObject();
				JsonObject subjectRotateCommand = subjects.getAsJsonArray().get(i).getAsJsonObject().get("rotate").getAsJsonObject();
				
				JsonObject pickCommand = new JsonObject();
				pickCommand.addProperty("pick" , "null");
				pickCommand.add("move" ,  subjectMoveCommand);
				pickCommand.add("rotate" ,  subjectRotateCommand);
				
				
				JsonObject pickRelativeTo = new JsonObject();
				pickRelativeTo.add(CompositeStep.LOOK_UP, subjects.getAsJsonArray().get(i).getAsJsonObject().get(CompositeStep.IDENTIFIER));

				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "command: " + pickCommand + ", relativeTo: " + pickRelativeTo);
				CompositeStep pick = new CompositeStep(productStep, pickCommand, pickRelativeTo);
				
				
				//Place
				JsonObject targetMoveCommand = target.getAsJsonObject().get("move").getAsJsonObject();
				JsonObject targetRotateCommand = target.getAsJsonObject().get("rotate").getAsJsonObject();
				
				JsonObject placeCommand = new JsonObject();
				placeCommand.addProperty("place", "null");
				placeCommand.add("move" ,  targetMoveCommand);
				placeCommand.add("rotate" ,  targetRotateCommand);
				
				JsonObject placeRelativeTo = new JsonObject();
				placeRelativeTo.add(CompositeStep.LOOK_UP, target.get(CompositeStep.IDENTIFIER));
				
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "command: " + placeCommand + ", relativeTo: " + placeRelativeTo);
				CompositeStep place = new CompositeStep(productStep, placeCommand, placeRelativeTo);
				
				
				//Translate to hardwareSteps
				ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
				ArrayList<CompositeStep> capabilities = new ArrayList<CompositeStep>();
				capabilities.add(pick);
				capabilities.add(place);
				hardwareSteps.addAll(translateCompositeStep(modules, capabilities));
				
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
			}
		}			
		return hardwareSteps;
	}
}