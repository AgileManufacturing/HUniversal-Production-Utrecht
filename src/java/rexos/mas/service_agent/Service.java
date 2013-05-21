/**
 * @file rexos/mas/service_agent/Service.java
 * @brief Instances of subclasses of this interface represent different services a certain equiplet can perform.
 *        Services translate product steps into service steps.
 * @date Created: 11 apr. 2013
 * 
 * @author Peter Bonnema
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht. All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met: - Redistributions of source code must retain the above copyright notice,
 *          this list of conditions and the following disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 *          other materials provided with the distribution. - Neither the name of the HU University of Applied Sciences
 *          Utrecht nor the names of its contributors may be used to endorse or promote products derived from this
 *          software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 *          WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE
 *          FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *          LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *          THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.service_agent;

import java.util.HashMap;

import rexos.mas.data.Position;

import com.mongodb.BasicDBObject;

/**
 * Instances of subclasses of this interface represent different services a certain equiplet can perform. Services
 * translate product steps into service steps.
 * 
 * @author Peter Bonnema
 * 
 */
public interface Service {
	/**
	 * This method determines which modules are needed to perform this service and returns an array containing the
	 * module id's of those modules.
	 * 
	 * @param productStepType The type of the productStep that needs to be translated
	 * @param parameters The parameters of the productStep.
	 * @return An array of module id's of the modules needed to perform the service.
	 */
	public int[] getModuleIds(int productStepType, BasicDBObject parameters);

	/**
	 * This method translates the productStep into a single or multiple serviceSteps.
	 * 
	 * @param productStepType The type of the productStep to translate.
	 * @param parameters The parameters of the productStep to use.
	 * @return An array of ServiceStep each representing a serviceStep as a result of the translation.
	 */
	public ServiceStep[] getServiceSteps(int productStepType, BasicDBObject parameters);

	/**
	 * Updates the parameters of the serviceSteps with new information contained in partParameters. partParameters
	 * should contain positional information about parts that was unknown when getServiceSteps was called. Get this
	 * additional information from the logistics agent.
	 * 
	 * @param partParameters the extra positional parameters to update the service steps with.
	 * @param serviceSteps the service steps to update.
	 * @return the updated service steps.
	 */
	public ServiceStep[] updateParameters(HashMap<Integer, Position> partParameters,
			ServiceStep[] serviceSteps);

	/**
	 * Returns the Id of this service. Id's are used to identify services and are the same as those used in the
	 * knowledge database.
	 * 
	 * @return the id of this service.
	 */
	public int getId();

	/**
	 * Implementations should return a clear name for this service analog to its function. It is intended for debugging
	 * purposes. Internally services are identified by their Id and not by their name.
	 * 
	 * @return the name of this service.
	 */
	public String getName();
}
