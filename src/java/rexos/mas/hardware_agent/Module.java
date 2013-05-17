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
	private int id;
	private ModuleFactory moduleFactory;
	private HashMap<Integer, Object> configuration;

	/**
	 * 
	 * @param parameters
	 * @return
	 */
	public abstract EquipletStepMessage[] getEquipletSteps(int stepType, BasicDBObject parameters);

	public abstract EquipletStepMessage[] fillPlaceHolders(EquipletStepMessage[] steps, BasicDBObject parameters);
	
	/**
	 * @return
	 */
	public abstract int[] isLeadingForSteps();
	
	/**
	 * @return
	 */
	public int getId() {
		return id;
	}
	
	/**
	 * @param id
	 */
	public void setId(int id) {
		this.id = id;
	}
	
	/**
	 * @return the moduleFactory
	 */
	public ModuleFactory getModuleFactory() {
		return moduleFactory;
	}

	/**
	 * @param moduleFactory the moduleFactory to set
	 */
	public void setModuleFactory(ModuleFactory moduleFactory) {
		this.moduleFactory = moduleFactory;
	}

	/**
	 * @return the configuration
	 */
	public HashMap<Integer, Object> getConfiguration() {
		return configuration;
	}

	/**
	 * @param configuration the configuration to set
	 */
	public void setConfiguration(HashMap<Integer, Object> configuration) {
		this.configuration = configuration;
	}
}
