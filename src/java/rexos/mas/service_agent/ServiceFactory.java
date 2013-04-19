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
package serviceAgent;

import java.io.IOException;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Hashtable;

import rexos.libraries.dynamicloader.DynamicClassDescription;
import rexos.libraries.dynamicloader.DynamicClassFactory;
import rexos.libraries.knowledge.Row;
import rexos.libraries.knowledge.KnowledgeDBClient;
import rexos.libraries.knowledge.Queries;

import jade.core.AID;

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
		KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
		try {
			ResultSet rs = knowledgeClient.executeSelectQuery("SELECT * FROM software WHERE id=5");
			if (rs.next()) {
				Row row = new Row(rs);
				DynamicClassDescription description = DynamicClassDescription.createFromRow(row);
				service = factory.createNewObjectIfOutdated(description, serviceCache.get(description.getId()));
				serviceCache.put(description.getId(), service);
			}
		} catch (Exception e) {
			//TODO: Do something useful.
			e.printStackTrace();
		}
		
		return service;
	}
	
	public Service[] getServicesForStep(long stepType) {
//		KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
//		ArrayList<Service> servicesForStep = new ArrayList<Service>();
//		try {
//			ResultSet rs = knowledgeClient.executeSelectQuery(
//					Queries.POSSIBLE_SERVICES_PER_EQUIPLET,
//					new Object[]{equipletAID, stepType});
//			
//			ArrayList<Integer> serviceIDs = new ArrayList<Integer>();
//			while (rs.next()) {
//				Row row = new Row(rs);
//				serviceIDs.add((Integer)row.get("id"));
//			}
//			
//			for (Integer serviceID : serviceIDs) {
//				
//			}
//		} catch (SQLException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
		return new Service[]{getServiceByServiceID(1)};
	}
}
