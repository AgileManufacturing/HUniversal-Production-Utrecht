/**
 * @file EquipletAgent.java
 * @brief Provides an equiplet agent that communicates with product agents and with its own service agent.
 * @date Created: 2013-04-02
 *
 * @author Hessel Meulenbeld
 * @author Thierry Gerritse
 * @author Wouter Veen
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

package equipletAgent;

import com.mongodb.*;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import newDataClasses.ParameterList;
import newDataClasses.ProductionStep;
import nl.hu.client.BlackboardClient;
import serviceAgent.ServiceAgent;

import java.io.IOException;
import java.io.Serializable;
import java.lang.reflect.Type;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Hashtable;

import org.bson.BSONObject;
import org.bson.types.ObjectId;

/**
 * EquipletAgent that communicates with product agents and with its own service
 * agent.
 **/
public class EquipletAgent extends Agent {
	private static final long serialVersionUID = 1L;

	// This is the service agent of this equiplet
	private AID serviceAgent;
	
	// This is the collective database used by all product agents and equiplets
	// and contains the collection EquipletDirectory.
	private DB collectiveDb = null;
	private String collectiveDbIp = "localhost";
	private int collectiveDbPort = 27017;
	private String collectiveDbName = "CollectiveDb";
	private String equipletDirectoryName = "EquipletDirectory";

	// This is the database specific for this equiplet, this database contains
	// all the collections of the equiplet, service and hardware agents.
	private DB equipletDb = null;
	private String equipletDbIp = "localhost";
	private int equipletDbPort = 27017;
	private String equipletDbName = "";
	private DBCollection productSteps = null;
	private String productStepsName = "ProductStepsBlackBoard";

	//BlackboardClient to communicate with the blackboards
	private BlackboardClient client;

	//Arraylist with IDs of the capabilities of the equiplet
	private ArrayList<Long> capabilities;

	private Hashtable<String, ObjectId> communicationTable;

	@SuppressWarnings("unchecked")
	public void setup() {
		System.out.println("I spawned as a equiplet agent.");
		
		// set the database name to the name of the equiplet and set the database ip to its own IP.
		try{
			InetAddress IP = InetAddress.getLocalHost();
			equipletDbIp = IP.getHostAddress();
		}catch(Exception e){
			e.printStackTrace();
		}
		equipletDbName = getAID().getLocalName();
		communicationTable = new Hashtable<String, ObjectId>();
		// TODO: Not Hardcoded capabilities/get capabilities from the service agent.
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			capabilities = (ArrayList<Long>) args[0];
			System.out.format("%s %s%n", capabilities, equipletDbName);
		}
        try {
        	DbData dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);
        	Object[] arguments = new Object[]{dbData};
			((AgentController)getContainerController().createNewAgent(getLocalName() + "-serviceAgent", "serviceAgent.ServiceAgent", arguments)).start();
			serviceAgent = new AID((String) getLocalName() + "-serviceAgent", AID.ISLOCALNAME);
		} catch (StaleProxyException e1) {
			e1.printStackTrace();
			doDelete();
		}
		
		
		
		Gson gson = new Gson();
		// put capabilities on the equipletDirectory
		try {
			client = new BlackboardClient(collectiveDbIp);
			client.setDatabase(collectiveDbName);
			client.setCollection(equipletDirectoryName);
			DbData dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);
			EquipletDirectoryMessage entry = new EquipletDirectoryMessage(getAID(), capabilities, dbData);
			client.insertDocument(gson.toJson(entry));
		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}
		

		// Behaviour for receiving messages, checks each 5000
		 addBehaviour(new CyclicBehaviour(this) {
			private static final long serialVersionUID = 1L;

			@Override
			public void action() {

				// myAgent.addBehaviour(new RequestPerformer());
				ACLMessage msg = receive();
				Gson gson;
				if (msg != null) {
					// Process the message

					// deserialize content
					Object contentObject = null;
					String contentString = msg.getContent();

					try {
						contentObject = msg.getContentObject();
					} catch (UnreadableException e) {
//						System.out.println("Exception Caught, No Content Object Given " + e);
					}

					System.out.format("%s received message from %s (%s:%s)%n",
							getLocalName(), msg.getSender().getLocalName(), msg.getOntology(),
							contentObject != null ? contentObject : contentString);

					// Start of the switch statement on the ontology
					switch (msg.getOntology()) {

					// Case to check if the equiplet can perform the given step.
					case "CanPerformStep":
						// getting the product step from the message.
						ProductionStep proStepC = (ProductionStep) contentObject;
						ObjectId productStepEntryId = null;
						gson = new GsonBuilder().serializeNulls().create();

						// Makes a database connection and puts the new step in it.
						//TODO make use of a single BlackBoardClient object in an instance variable instead of recreating it
						try {
							client = new BlackboardClient(equipletDbIp);
							client.setDatabase(equipletDbName);
							client.setCollection(productStepsName);
							// TODO: get inputParts
							// TODO: get ouputPart
							ProductStepMessage entry = new ProductStepMessage(msg.getSender(), proStepC.getCapability(),
									proStepC.getParameterList(), null, null,
									ProductStepStatusCode.EVALUATING.getStatus(), null);
							productStepEntryId = client.insertDocument(gson.toJson(entry));
							communicationTable.put(msg.getConversationId(), productStepEntryId);
							// Asks the serviceAgent if it can do this product step.
							ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
							message.setConversationId(msg.getConversationId());
							message.addReceiver(serviceAgent);
							message.setOntology("canDoProductionStep");
							try {
								message.setContentObject(productStepEntryId);
							} catch (IOException e) {
								e.printStackTrace();
								// TODO: ERROR HANDLING'
								myAgent.doDelete();
							}
							send(message);
						} catch (Exception e) {
							e.printStackTrace();
							// TODO: ERROR HANDLING
							myAgent.doDelete();
						}
						break;
					// Case for the response on the send message to the service
					// agent in the case canDoProductionStep
					case "canDoProductionStepResponse":
						productStepEntryId = null;
						DBObject productStep = null;
						gson = new GsonBuilder()
							.registerTypeAdapter(jade.util.leap.List.class, new InstanceCreator<jade.util.leap.List>() {
								@Override
								public jade.util.leap.List createInstance(Type type) {
									return new jade.util.leap.ArrayList();
								}
							})
							.create();
						//TODO: cleanup onderstaande
						//TODO: one blackboardclient
						try {
							productStepEntryId = (ObjectId) contentObject;
						} catch (ClassCastException e) {
							// TODO: ERROR HANDLING
							e.printStackTrace();
							myAgent.doDelete();
						}
						// Makes a database connection and gets the right
						// product step out of it.
						client = new BlackboardClient(equipletDbIp);
						try {
							client.setDatabase(equipletDbName);
							client.setCollection(productStepsName);
							BasicDBObject query = new BasicDBObject();
							query.put("_id", productStepEntryId);
							productStep = client.findDocuments(query).get(0);
							int status = (Integer) productStep.get("status");
							AID productAgent = gson.fromJson(productStep.get("productAgentId").toString(), AID.class);
							if (status == ProductStepStatusCode.EVALUATING.getStatus()) {
								ACLMessage message = new ACLMessage(ACLMessage.CONFIRM);
								message.setConversationId(msg.getConversationId());
								message.setOntology("");// TODO: set ontology
								message.addReceiver(productAgent);
								message.setContent("This is possible");
								send(message);
							} else if (status == ProductStepStatusCode.ABORTED.getStatus()) {
								ACLMessage message = new ACLMessage(ACLMessage.DISCONFIRM);
								message.setConversationId(msg.getConversationId());
								message.setOntology("");// TODO: set ontology
								message.addReceiver(productAgent);
								message.setContent("This is impossible");
								send(message);
							}
						} catch (Exception e) {
							// TODO: ERROR HANDLING
							e.printStackTrace();
							myAgent.doDelete();
						}
						break;

					case "GetProductionDuration":

						try {
							ObjectId contentObjectId = communicationTable.get(msg.getConversationId());
							ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
							message.addReceiver(serviceAgent);
							message.setConversationId(msg.getConversationId());
							message.setContentObject(contentObjectId);
							message.setOntology("getProductionStepDuration");

//							try {
//
//								message.setContentObject((Serializable) contentObject);
//
//							} catch (IOException e) {
//
//								System.out.println(e);
//								e.printStackTrace();
//
//							}

							System.out.format("%s sending message: %s:%n",
									getLocalName(), message.getOntology(),
									message.getContentObject() != null ? message.getContentObject() : message.getContent());
							send(message);

						} catch (Exception e) {
							// TODO: ERROR HANDLING
							e.printStackTrace();
							myAgent.doDelete();
						}

						break;
					case "ProductionDurationResponse":
						try {
							gson = new GsonBuilder()
								.registerTypeAdapter(jade.util.leap.List.class, new InstanceCreator<jade.util.leap.List>() {
									@Override
									public jade.util.leap.List createInstance(Type type) {
										return new jade.util.leap.ArrayList();
									}
								}).create();
							ObjectId contentObjectId = communicationTable.get(msg
									.getConversationId());
							client = new BlackboardClient(equipletDbIp);
						
							client.setDatabase(equipletDbName);
							client.setCollection(productStepsName);
							BasicDBObject query = new BasicDBObject();
							query.put("_id", contentObjectId);
							productStep = client.findDocuments(query).get(0);
														
							ScheduleData Schedule = gson.fromJson(productStep.get("scheduleData").toString(), ScheduleData.class);
							
							ACLMessage message = new ACLMessage(ACLMessage.INFORM);
							message.addReceiver(gson.fromJson(productStep.get("productAgentId").toString(), AID.class));
							message.setOntology("ProductionDuration");
							message.setConversationId(msg.getConversationId());
							message.setContentObject(Schedule.getDuration());
							
							System.out.format("%s sending message: %s:%n",
									getLocalName(), message.getOntology(),
									message.getContentObject() != null ? message.getContentObject() : message.getContent());
							send(message);
							
						}catch(Exception e){
							e.printStackTrace();
							myAgent.doDelete();
						}
					
						break;

					case "scheduleStep":
						try{
							
							long timeslot = Long.parseLong(contentString);
							ObjectId contentObjectId = communicationTable.get(msg
									.getConversationId());
							client = new BlackboardClient(equipletDbIp);
							client.setDatabase(equipletDbName);
							client.setCollection(productStepsName);
							BasicDBObject query = new BasicDBObject();
							query.put("_id", contentObjectId);
							productStep = client.findDocuments(query).get(0);
							System.out.format("%d%n", timeslot);
							ACLMessage timeslotMessage = new ACLMessage(
									ACLMessage.REQUEST);
							timeslotMessage.addReceiver(serviceAgent);
							timeslotMessage.setOntology("scheduleStepWithLogistics");
							timeslotMessage.setContent(String.valueOf(timeslot));
							timeslotMessage.setConversationId(msg.getConversationId());
							send(timeslotMessage);
						/*
						 * TODO: Ask service agent to schedule the step with the
						 * logistics at time X if possible. Wait for result.
						 * Report result back to product agent. If the result is
						 * positive: Set the status of the step on the product
						 * steps blackboard to PLANNED and add the schedule
						 * data.
						 */
							ACLMessage confirmScheduleStep = new ACLMessage(ACLMessage.CONFIRM);
							confirmScheduleStep.setConversationId(msg.getConversationId());
							confirmScheduleStep.addReceiver((AID) productStep.get("productAgentId"));
						}
						catch(Exception e){
							e.printStackTrace();
						}
						break;
					}

				}
				block();
			}
		});
	}
	public void takeDown() {
		Gson gson = new Gson();
		try {
			client = new BlackboardClient(collectiveDbIp);
			try {
				client.setDatabase(collectiveDbName);
				client.setCollection(equipletDirectoryName);

				BasicDBObject searchQuery = new BasicDBObject();
				searchQuery.put("AID", getAID());

				client.removeDocuments(gson.toJson(searchQuery));
			} catch (Exception e) {
				e.printStackTrace();
				// The equiplet is already going down, so it has to do nothing
				// here.
			}
			// TODO: message to PA's

			/*Mongo equipletDbMongoClient = new Mongo(equipletDbIp, equipletDbPort);
			equipletDb = equipletDbMongoClient.getDB(equipletDbName);
			equipletDb.dropDatabase();
			equipletDbMongoClient.close();*/
		} catch (Exception e) {
			e.printStackTrace();
			// The equiplet is already going down, so it has to do nothing here.
		}
	}
}
