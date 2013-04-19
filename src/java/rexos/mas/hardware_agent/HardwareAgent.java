package rexos.mas.hardware_agent;

/**
 * @file HardwareAgent.java
 * @brief Provides an Hardware agent that communicates with Service agents and its own modules.
 * @date Created: 12-04-13
 *
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
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

import java.util.HashMap;

import rexos.mas.data.DbData;
import rexos.mas.hardware_agent.behaviours.*;
import rexos.mas.service_agent.Service;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import jade.core.Agent;
import rexos.libraries.blackboard_client.BasicOperationSubscription;
import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;

public class HardwareAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;

	private BlackboardClient serviceStepBBClient, equipletStepBBClient;
	private DbData dbData;
	private HashMap<Long,Module> ModulesMap;
	private HashMap<Service, Module> LeadingModules;
		
	public void RegisterModule(long id,Module module) {
		this.ModulesMap.put(id, module);
	}
	
	public Module GetModuleById(long id) {
		return ModulesMap.get(id);
	}
	
	@Override
	public void setup() {
		System.out.println("Hardware agent "+ this +" reporting.");
		ModulesMap = new HashMap<Long,Module>();
		LeadingModules = new HashMap<Service, Module>();

		// TODO fill in host, database and collection
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
		}

		try {
			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");
			serviceStepBBClient.subscribe(new BasicOperationSubscription(MongoOperation.INSERT, this));
			serviceStepBBClient.subscribe(new BasicOperationSubscription(MongoOperation.UPDATE, this));
			
			equipletStepBBClient = new BlackboardClient(dbData.getIp());
			equipletStepBBClient.setDatabase(dbData.getName());
			equipletStepBBClient.setCollection("EquipletStepsBlackBoard");
			equipletStepBBClient.subscribe(new BasicOperationSubscription(MongoOperation.UPDATE, this));
		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}
		
		EvaluateDuration evaluateDurationBehaviour = new EvaluateDuration(this);
		addBehaviour(evaluateDurationBehaviour);
		
		FillPlaceholders fillPlaceholdersBehaviour = new FillPlaceholders(this);
		addBehaviour(fillPlaceholdersBehaviour);
		
		CheckForModules checkForModules = new CheckForModules(this);
		addBehaviour(checkForModules);
		
		///Register modules
		
		/// modulefactory aan het werk gezet
		
		
		// for now: use precompiled grippermodule class
		GripperModule gp = new GripperModule();
		RegisterModule(1l, gp);
		DeltaRobotModule drm = new DeltaRobotModule();
		RegisterModule(2l, drm);
		///
		
		
	}

	@Override
	public void takeDown() {
		// TODO implement graceful death
	}

	public BlackboardClient getServiceStepsBBClient(){
		return serviceStepBBClient;
	}
	
	
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		switch (entry.getNamespace().split(".")[1]) {
		case "ServiceStepsBlackboard":
			switch (operation) {
			case INSERT:
						
				break;
				//$CASES-OMITTED$
			default:
				break;
			}
			break;
		case "EquipletStepsBlackboard":
			switch(operation){
			case UPDATE:
				try {
					ObjectId id = entry.getTargetObjectId();
					BasicDBObject query = new BasicDBObject();
					query.put("_id", id);
					DBObject equipletStep = equipletStepBBClient.findDocuments(query).get(0);
				} catch (InvalidDBNamespaceException | GeneralMongoException e) {
					// TODO Error no document
					e.printStackTrace();
				}
				break;
				//$CASES-OMITTED$
			default:
				break;
			}
			break;
		default:
			break;
		}
	}
}
