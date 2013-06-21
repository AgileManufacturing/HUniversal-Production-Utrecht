/**
 * @file rexos/mas/service_agent/Service.java
 * @brief A service object represents a service an equiplet can perform. Services translate productSteps into
 *        serviceSteps.
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

import rexos.mas.data.Part;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;
import rexos.mas.data.Position;

import com.mongodb.BasicDBObject;

/**
 * A service object represents a service an equiplet can perform. Services translate productSteps into serviceSteps.
 * 
 * @author Peter Bonnema
 * 
 */
public abstract class Service {
	/**
	 * @var String NAME
	 *      The name of the service. It should be the same as the one in the knowledge database.
	 */
	private String NAME;

	/**
	 * @var int ID
	 *      The id of the service. It should be the same as the one in the knowledge database.
	 */
	private int ID;

	/**
	 * Creates a new Service object with the specified name and id. Subclasses should supply a hard-coded name and id of
	 * the service to this constructor instead of asking it from client code.
	 * 
	 * @param name the name of this service
	 * @param id the id of this service
	 */
	protected Service(String name, int id) {
		NAME = name;
		ID = id;
	}

	/**
	 * This method determines what kind of modules (module groups) are needed to perform this service and returns an
	 * array containing the the id's of those modulegroups.
	 * 
	 * @param productStepType the type of the productStep that needs to be translated
	 * @param parameters the parameters of the productStep.
	 * @return an array of module id's of the modules needed to perform the service.
	 */
	public int[] getModuleGroupIds(int productStepType, BasicDBObject parameters) {
		try {
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] moduleGroups = client.executeSelectQuery(Queries.MODULEGROUPS_REQUIRED_PER_SERVICE, NAME);
			int[] moduleIds = new int[moduleGroups.length];
			for(int i = 0; i < moduleGroups.length; i++) {
				moduleIds[i] = (int) moduleGroups[i].get("module_id");
			}
			return moduleIds;
		} catch(KnowledgeException | KeyNotFoundException e) {
			Logger.log(e);
		}
		return null;
	}

	/**
	 * Determines whether this equiplet can perform a productStep of the specified type with the specified parameters
	 * using this service.
	 * 
	 * @param productStepType the type of the productStep to perform.
	 * @param parameters the parameters of the productStep.
	 * @return a boolean to indicate whether this equiplet can perform a productstep of the specified type with the
	 *         specified parameters using this service.
	 */
	public abstract boolean canDoStep(int productStepType, BasicDBObject parameters);

	/**
	 * This method translates the productStep into a single or multiple serviceSteps.
	 * 
	 * @param productStepType the type of the productStep to translate.
	 * @param parameters the parameters of the productStep to use.
	 * @return an array of ServiceStep each representing a serviceStep as a result of the translation.
	 */
	public abstract ServiceStep[] getServiceSteps(int productStepType, BasicDBObject parameters);

	/**
	 * Updates the parameters of the serviceSteps with new information contained in partParameters. partParameters
	 * should contain positional information about parts that was unknown when getServiceSteps was called. Get this
	 * additional information from the logistics agent.
	 * 
	 * @param partParameters the extra positional parameters to update the service steps with.
	 * @param serviceSteps the service steps to update.
	 * @return the updated service steps.
	 */
	public abstract ServiceStep[] updateParameters(HashMap<Part, Position> partParameters, ServiceStep[] serviceSteps);

	/**
	 * Returns the Id of this service. Id's are used to identify services and are the same as those used in the
	 * knowledge database.
	 * 
	 * @return the id of this service.
	 */
	public int getId() {
		return ID;
	}

	/**
	 * Implementations should return a clear name for this service analog to its function. It is intended for debugging
	 * purposes. Internally services are identified by their id and not by their name.
	 * 
	 * @return the name of this service.
	 */
	public String getName() {
		return NAME;
	}
}
