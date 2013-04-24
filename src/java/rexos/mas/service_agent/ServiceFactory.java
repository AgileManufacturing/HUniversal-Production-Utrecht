/**
 * @file ServiceFactory.java
 * @brief 
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

/**
 * 
 **/
public class ServiceFactory {
	private DynamicClassFactory<Service>factory;
	private Hashtable<Long, Service> serviceCache;
	private String equipletAID;
	
	public ServiceFactory(String equipletAID) {
		this.equipletAID = equipletAID;
		serviceCache = new Hashtable<Long, Service>();
		this.factory = new DynamicClassFactory<Service>(Service.class);
	}
	
	private Service	getServiceByServiceID(int serviceID) {
		Service service = null;
		try {
			KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
			Row[] rows = knowledgeClient.executeSelectQuery("SELECT * FROM software WHERE id=5");
			if (rows.length > 0) {
				DynamicClassDescription description = DynamicClassDescription.createFromRow(rows[0]);
				service = factory.createNewObjectIfOutdated(description, serviceCache.get(description.getId()));
				serviceCache.put(description.getId(), service);
			}
		} catch (KnowledgeException | InstantiateClassException | KeyNotFoundException e) {
			//TODO: Do something useful.
			e.printStackTrace();
		}
		
		return service;
	}
	
	public Service[] getServicesForStep(int stepType) {
		ArrayList<Service> servicesForStep = new ArrayList<Service>();
		try {
			KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
			Row[] rows = knowledgeClient.executeSelectQuery(
					Queries.SERVICES_FOR_STEP_FOR_EQUIPLET,
					equipletAID, stepType);
			
			for (int i = 0 ; i < rows.length ; ++i) {
				servicesForStep.add(getServiceByServiceID((int)rows[i].get("id")));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			// TODO Auto-generated catch block
			ex.printStackTrace();
		}
		
		Service[] services = new Service[servicesForStep.size()];
		servicesForStep.toArray(services);
		return services;
	}
}
