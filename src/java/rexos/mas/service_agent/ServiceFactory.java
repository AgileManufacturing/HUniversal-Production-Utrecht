/**
 * @file rexos/mas/service_agent/ServiceFactory.java
 * @brief Helper class for creating Service objects based on knowledgebase data.
 * @date Created: 12 apr. 2013
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package rexos.mas.service_agent;

import java.util.ArrayList;
import java.util.Hashtable;

import rexos.libraries.dynamicloader.DynamicClassDescription;
import rexos.libraries.dynamicloader.DynamicClassFactory;
import rexos.libraries.dynamicloader.InstantiateClassException;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.utillities.log.LogLevel;
import rexos.utillities.log.Logger;

/**
 * Helper class for creating Service objects based on knowledgebase data.
 **/
public class ServiceFactory {
	/**
	 * @var DynamicClassFactory<Service> factory
	 * The DynamicClassFactory used to instantiate Services.
	 */
	private DynamicClassFactory<Service>factory;
	
	/**
	 * Hashtable<Long, Service> serviceCache
	 * A cache of instantiated services. This cache stores the latest instantiated version of a Service.
	 */
	private Hashtable<Long, Service> serviceCache;
	
	/**
	 * @var String equipletAID
	 * The AID of the equiplet that will be used to determine which services are available.
	 */
	private String equipletAID;
	
	/**
	 * Constructs a new ServiceFactory for the given equiplet.
	 * @param equipletAID AID of the equiplet.
	 */
	public ServiceFactory(String equipletAID) {
		this.equipletAID = equipletAID;
		serviceCache = new Hashtable<Long, Service>();
		this.factory = new DynamicClassFactory<Service>(Service.class);
	}
	
	/**
	 * Returns a Service object for the given serviceID.
	 * @param serviceID The serviceID for which software should be loaded.
	 * @return The Service object for the given serviceID.
	 */
	private Service	getServiceByServiceID(int serviceID) {
		Service service = null;
		try {
			KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
			Row[] rows = knowledgeClient.executeSelectQuery(Queries.SOFTWARE_FOR_SERVICE, serviceID);
			if (rows.length > 0) {
				DynamicClassDescription description = new DynamicClassDescription(
						new Long((Integer)rows[0].get("id")),
						(String)rows[0].get("name"),
						(String)rows[0].get("description"),
						(String)rows[0].get("class_name"),
						(String)rows[0].get("jar_location"));
				service = factory.createNewObjectIfOutdated(description, serviceCache.get(description.getId()));
				service.setId(serviceID);
				service.setName(description.getName());
				serviceCache.put(description.getId(), service);
			}
		} catch (KnowledgeException | InstantiateClassException | KeyNotFoundException e) {
			Logger.log(LogLevel.ERROR, e);
		}
		
		return service;
	}
	
	/**
	 * Returns an array of Service objects for the services that are capable of processing the given product step.
	 * @param stepType The type of the product step.
	 * @return An array of Service objects capable of processing the given product step.
	 */
	public Service[] getServicesForStep(int stepType) {
		ArrayList<Service> servicesForStep = new ArrayList<Service>();
		try {
			KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
			Row[] rows = knowledgeClient.executeSelectQuery(
					Queries.SERVICES_FOR_STEP_FOR_EQUIPLET,
					equipletAID, stepType);
			
			for (int i = 0 ; i < rows.length ; ++i) {
				
				if(getServiceByServiceID((int)rows[i].get("id")) != null){
				
					servicesForStep.add(getServiceByServiceID((int)rows[i].get("id")));
					
				}
			}
		} catch (KnowledgeException | KeyNotFoundException e) {
			Logger.log(LogLevel.ERROR, e);
		}
		
		Service[] services = new Service[servicesForStep.size()];
		servicesForStep.toArray(services);
		return services;
	}
}
