/**
 * @file rexos/mas/hardware_agent/Module.java
 * @brief Provides an inteface to make modules.
 * @date Created: 2013-04-18
 * 
 * @author Thierry Gerritse
 * @author Peter Bonnema
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

import java.util.HashMap;

import com.mongodb.BasicDBObject;

public abstract class Module {
	/**
	 * @var int id
	 * The id of the module
	 */
	private int id;
	
	/**
	 * @var ModuleFactory moduleFactory
	 * The moduleFactory of the module.
	 */
	private ModuleFactory moduleFactory;
	
	/**
	 * @var HashMap<Integer, Object> configuration
	 * The configuration of the module.
	 */
	private HashMap<Integer, Object> configuration;

	/**
	 * Function for getting the EquipletSteps for a stepType.
	 * @param stepType The stepType to get the EquipletSteps for.
	 * @param parameters The parameters to use by making the steps.
	 * @return The EquipletStepMessages
	 */
	public abstract EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters);

	/**
	 * Function for filling in the placeholders.
	 * @param steps The steps to fill the placeholders for.
	 * @param parameters The parameters to fill in the placeholders.
	 * @return The filled in EquipletStepMessages
	 */
	public abstract EquipletStepMessage[] fillPlaceHolders(EquipletStepMessage[] steps, BasicDBObject parameters);
	
	/**
	 * Function for getting the steps this module is leading for.
	 * @return int[] with step ids.
	 */
	public abstract int[] isLeadingForSteps();
	
	/**
	 * Getter for the id.
	 * @return The id.
	 */
	public int getId() {
		return id;
	}
	
	/**
	 * Setter for the id.
	 * @param id The id to set it to.
	 */
	public void setId(int id) {
		this.id = id;
	}
	
	/**
	 * Getter for the moduleFactory
	 * @return the moduleFactory
	 */
	public ModuleFactory getModuleFactory() {
		return moduleFactory;
	}

	/**
	 * Setter for the moduleFactory
	 * @param moduleFactory the moduleFactory to set it to
	 */
	public void setModuleFactory(ModuleFactory moduleFactory) {
		this.moduleFactory = moduleFactory;
	}

	/**
	 * Getter for the configuration
	 * @return the configuration
	 */
	public HashMap<Integer, Object> getConfiguration() {
		return configuration;
	}

	/**
	 * Setter for the configuration
	 * @param configuration the configuration to set it to
	 */
	public void setConfiguration(HashMap<Integer, Object> configuration) {
		this.configuration = configuration;
	}
}
