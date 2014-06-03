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

import java.util.ArrayList;

import libraries.dynamicloader.JarFileLoaderException;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

/**
 * Pick and place capability class that translate with rotation.
 * 
 *
 */
public class PickAndPlaceWithRotation extends Capability {
	
	/**
	 * 
	 * @param moduleFactory
	 */
	public PickAndPlaceWithRotation(ModuleFactory moduleFactory) {
		super(moduleFactory, "Place");
	}

	/**
	 * @see Capability#translateProductStep(ProductStep)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws FactoryException, JarFileLoaderException, CapabilityException {
		// TODO Auto-generated method stub
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonObject target = productStepCriteria.get("target").getAsJsonObject();
		JsonArray subjects = productStepCriteria.get("subjects").getAsJsonArray();
		
		
		if(serviceName.equals("place") && subjects != null && target != null){
			JsonObject pickCommand = new JsonObject();
			JsonObject placeCommand = new JsonObject();
			for(int i = 0; i<subjects.getAsJsonArray().size();i++){
				JsonObject subjectMoveCommand = subjects.get(i).getAsJsonObject().get("move").getAsJsonObject();
				
				pickCommand.addProperty("pick" , "null");
				pickCommand.add("move" ,  subjectMoveCommand);
				pickCommand.addProperty("look_up", subjects.get(i).getAsJsonObject().get("identifier").getAsString());
				
				CompositeStep pick = new CompositeStep(productStep, pickCommand);
				
				JsonObject targetMoveCommand = target.getAsJsonObject().get("move").getAsJsonObject();
				placeCommand.addProperty("place", "null");
				placeCommand.add("move" ,  targetMoveCommand);
				placeCommand.addProperty("look_up", target.get("identifier").getAsString());

				CompositeStep place = new CompositeStep(productStep, placeCommand);
				
				ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
				
				for (ModuleActor moduleActor : modules) {
					try {
						
						hardwareSteps.addAll(moduleActor.translateCompositeStep(pick));
						hardwareSteps.addAll(moduleActor.translateCompositeStep(place));
						
					} catch (ModuleTranslatingException ex) {
						
						throw new CapabilityException(ex.toString(), ex);
					}
					
				}
				System.out.println(hardwareSteps.toString());

			}
		}
			
		return hardwareSteps;
	}

}