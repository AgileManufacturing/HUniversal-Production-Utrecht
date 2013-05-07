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

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import java.sql.ResultSet;
import java.util.HashMap;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BasicOperationSubscription;
import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.FieldUpdateSubscription;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;
import rexos.libraries.blackboard_client.FieldUpdateSubscription.MongoUpdateLogOperation;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.mas.data.DbData;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.hardware_agent.behaviours.CheckForModules;
import rexos.mas.hardware_agent.behaviours.EvaluateDuration;
import rexos.mas.hardware_agent.behaviours.FillPlaceholders;
import rexos.mas.service_agent.ServiceStepMessage;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class HardwareAgent extends Agent implements BlackboardSubscriber, ModuleUpdateListener {
	private static final long serialVersionUID = 1L;

	private BlackboardClient serviceStepBBClient, equipletStepBBClient;
	private DbData dbData;
	private HashMap<Integer, Module> Modules;
	private HashMap<Integer, Integer> leadingModuleForStep;
	
	public void registerModule(int serviceId, Module module) {
		Modules.put(serviceId, module);
	}

	public Module getModule(int serviceId) {
		return Modules.get(serviceId);
	}

	@Override
	public void setup() {
		System.out.println("Hardware agent " + this + " reporting.");
		Modules = new HashMap<Integer, Module>();

		// TODO fill in host, database and collection
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
		}

		try {
			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");

			FieldUpdateSubscription statusSubscription = new FieldUpdateSubscription(
					"status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			serviceStepBBClient.subscribe(statusSubscription);

			equipletStepBBClient = new BlackboardClient(dbData.getIp());
			equipletStepBBClient.setDatabase(dbData.getName());
			equipletStepBBClient.setCollection("EquipletStepsBlackBoard");
			equipletStepBBClient.subscribe(new BasicOperationSubscription(
					MongoOperation.UPDATE, this));
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

		// /Register modules
		KnowledgeDBClient client;
		try {
			client = KnowledgeDBClient.getClient();

			Row[] resultSet;

			resultSet = client.executeSelectQuery(Queries.MODULES);

			for (int i = 0; i < resultSet.length; i++) {
				System.out.println(resultSet[i]);

				// neem id van module
				// geef id aan modulefactory,
				// je krijgt een module terug,
				//
				// register module bij de hardwareagent..

			}
			System.out.println();

		} catch (KnowledgeException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		// kijk voor alle modules in de hashmap modules voor welke stap/stappen
		// deze module leidend is
		// en zet het id van de module, en het id van de bijbehorende step in de
		// leadingModuleForStep Hashmap

		// for now: use precompiled grippermodule class

		//GripperModule gp = new GripperModule();
		//registerModule(1, gp);
		//DeltaRobotModule drm = new DeltaRobotModule();
		//registerModule(2, drm);
		///		
		
		/// ga na voor welke stappen deze modules leidend zijn en sla dit op in de hashmap

	}
	
	public Module getLeadingModuleForStep(int stepId){
	
		int moduleId = leadingModuleForStep.get(stepId);
		
		return getModule(moduleId);
	}
	@Override
	public void takeDown() {
		// TODO implement graceful death
	}

	public BlackboardClient getServiceStepsBBClient() {
		return serviceStepBBClient;
	}

	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		switch (entry.getNamespace().split("\\.")[1]) {
		case "ServiceStepsBlackboard":
			switch (operation) {
			case UPDATE:

				ObjectId id = entry.getTargetObjectId();
				try {

					ServiceStepMessage serviceStep = new ServiceStepMessage(
							(BasicDBObject) serviceStepBBClient
									.findDocumentById(id));

					Module leadingModule = getModule(serviceStep.getServiceId());

					EquipletStepMessage[] equipletSteps = leadingModule
							.getEquipletSteps(serviceStep.getType(),
									serviceStep.getParameters());

					for (EquipletStepMessage eq : equipletSteps) {

						equipletStepBBClient.insertDocument(eq
								.toBasicDBObject());

					}

				} catch (InvalidDBNamespaceException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (GeneralMongoException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}

				break;
			default:
				break;
			}
			break;
		case "EquipletStepsBlackboard":
			switch (operation) {
			case UPDATE:
				try {
					ObjectId id = entry.getTargetObjectId();
					BasicDBObject query = new BasicDBObject();
					query.put("_id", id);
					DBObject equipletStep = equipletStepBBClient.findDocuments(
							query).get(0);
				} catch (InvalidDBNamespaceException | GeneralMongoException e) {
					// TODO Error no document
					e.printStackTrace();
				}
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	}

	@Override
	public void onModuleUpdate(int moduleId, Module oldSoftware, Module newSoftware) {
		try {
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] rows;

			rows = client.executeSelectQuery(Queries.MODULES);

			for (int i = 0; i < rows.length; i++) {
				System.out.println(rows[i]);

				// neem id van module
				// geef id aan modulefactory,
				// je krijgt een module terug,
				//
				// register module bij de hardwareagent..

			}
		} catch (KnowledgeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
