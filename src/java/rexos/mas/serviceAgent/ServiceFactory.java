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
package rexos.mas.serviceAgent;

import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Hashtable;

import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.Queries;

import jade.core.AID;

/**
 * 
 **/
public class ServiceFactory {
	private Hashtable<Integer, Service> serviceCache;
	private String equipletAID;
	
	public ServiceFactory(String equipletAID) {
		this.equipletAID = equipletAID;
	}
	
	private Service	getServiceByServiceID(int serviceID) {
		if (serviceCache.containsKey(serviceID)) {
			
		}
		
		return null;
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
		
		return new Service[]{new DummyService()};
	}
}
