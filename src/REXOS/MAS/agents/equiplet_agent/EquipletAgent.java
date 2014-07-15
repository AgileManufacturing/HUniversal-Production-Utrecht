/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	EquipletAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief  The EquipletAgent that is responsible for the communication between the Equiplet and the ProductAgent.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-05-20
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Tom Oosterwijk
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2014, HU University of Applied Sciences Utrecht. 
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package agents.equiplet_agent;

import generic.ProductStep;
import generic.Service;

import java.util.ArrayList;
import java.util.Date;

import agents.data_classes.MessageType;
import agents.equiplet_agent.reconfigure.behaviours.ReconfigureBehaviour;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;




//import wsig.agent.Reconfigure;
//import wsig.agent.ReconfigureOntology;
import agents.equiplet_agent.ReconfigureOntology;
import HAL.HardwareAbstractionLayer;
import HAL.steps.HardwareStep;
import HAL.Module;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.FactoryException;
import HAL.listeners.HardwareAbstractionLayerListener;
import jade.content.AgentAction;
import jade.content.ContentElement;
import jade.content.lang.sl.SLCodec;
import jade.content.onto.basic.Action;
import jade.content.onto.basic.Done;
import jade.content.onto.basic.Result;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPANames;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.FIPAManagementOntology;
import jade.domain.FIPAAgentManagement.Property;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KnowledgeException;

public class EquipletAgent extends Agent implements
		HardwareAbstractionLayerListener {

	/**
	 * @var machineState Stores the current state that the equiplet is in. This
	 *      variable is used in the ReconfigureBehaviour to start the
	 *      reconfigServer.
	 */
	public static String machineState = "";

	/**
	 * @var machineMode Stores the current mode that the equiplet is in. This
	 *      variable is used in the ReconfigureBehaviour to start the
	 *      reconfigServer.
	 */
	public static String machineMode = "";

	private static final long serialVersionUID = -4551409467306407788L;

	/**
	 * @var equipletSchedule The ArrayList equipletSchedules holds all the Job
	 *      objects that the EQ needs to execute.
	 */
	private ArrayList<Job> equipletSchedule = new ArrayList<Job>();

	/**
	 * @var serviceList The ArrayList serviceList holds all the services that
	 *      the equiplet can execute.
	 */
	private ArrayList<Service> serviceList = null;

	/**
	 * @var scheduleCounter Counts the amount of schedules that have been
	 *      executed.
	 */
	private int scheduleCounter = 0;

	/**
	 * @var hal The Object HAL is used to execute the planned jobs.
	 */
	private HardwareAbstractionLayer hal;

	/**
	 * @var serverLists The string serverLists Holds all the services that the
	 *      equiplet can exectute. This variable is used in the WIMP.
	 */
	public String serverLists = "";

	// TODO hardcoded equipletName !
	/**
	 * @var EQName Hardcoded EquipletName. This should be changed in the future.
	 */
	private String EQName = "EQ3";

	/**
	 * @var equipletActive The boolean equipletActive keeps track wether or not
	 *      the equiplet is currently in use or not. This information is
	 *      displayed in the WIMP.
	 */
	private boolean equipletActive = false;

	/**
	 * @var productStepCounter The productStepCounter keeps track off all the
	 *      planned productSteps. This information is displayed in the WIMP.
	 */
	private int productStepCounter = 0;

	/**
	 * @var productStepFailedCounter The productStepFailedCounter keeps track
	 *      off all the failed productSteps. This information is displayed in
	 *      the WIMP.
	 */
	private int productStepFailedCounter = 0;

	/**
	 * @var productStepSuccesCounter The productStepSuccesCounter keeps track
	 *      off all the successfully executed productSteps. This information is
	 *      displayed in the WIMP.
	 */
	private int productStepSuccesCounter = 0;

	public static final String WSIG_FLAG = "wsig";
	public static final String WSIG_MAPPER = "wsig-mapper";
	public static final String WSIG_PREFIX = "wsig-prefix";
	public static AID myAID = null;
	private SLCodec codec = new SLCodec();
	private Date startDate;
	private ReconfigureBehaviour reconf;
	private MessageTemplate template = MessageTemplate
			.MatchOntology(ReconfigureOntology.getInstance().getName());

	/**
	 * setup() The function setup creates the HAL object and initialized the
	 * serviceList. The EquipletAgent then registers himself, with his service
	 * types to the DF. Main task is to wait for incoming ACLMessaged to
	 * communicate with ProductAgents.
	 */
	protected void setup() {
		System.out.println("setup equipletAgent");

		// Register codec/onto
		getContentManager().registerLanguage(codec);
		getContentManager().registerOntology(
				FIPAManagementOntology.getInstance());
		getContentManager().registerOntology(ReconfigureOntology.getInstance());

		// Prepare a DFAgentDescription
		DFAgentDescription dfad = new DFAgentDescription();
		dfad.setName(this.getAID());
		dfad.addLanguages(codec.getName());
		dfad.addProtocols(FIPANames.InteractionProtocol.FIPA_REQUEST);
		ServiceDescription sd;
		sd = new ServiceDescription();
		sd.addLanguages(codec.getName());
		sd.addProtocols(FIPANames.InteractionProtocol.FIPA_REQUEST);
		sd.setType("ReconfigureAgent");
		sd.setOwnership("ReconfigureOwner");
		sd.addOntologies(ReconfigureOntology.getInstance().getName());

		// WSIG properties
		sd.addProperties(new Property(WSIG_FLAG, "true"));
		Object[] args = getArguments();

		for (int i = 0; i < args.length; i++)
			System.out.println(args[i].toString());

		// Service name
		String wsigServiceName = "Reconfigure";
		// if (args.length >= 1) {
		// wsigServiceName = (String) args[0];
		// }
		System.out.println("Service name: " + wsigServiceName);
		sd.setName(wsigServiceName);

		// Mapper
		boolean isMapperPresent = false;
		if (args.length >= 2) {
			isMapperPresent = Boolean.parseBoolean((String) args[1]);
		}
		System.out.println("Mapper present: " + isMapperPresent);
		if (isMapperPresent) {
			sd.addProperties(new Property(WSIG_MAPPER,
					"agents.equiplet_agent.ReconfigureOntology"));
		}

		// Prefix
		String wsigPrefix = "";
		if (args.length >= 3) {
			wsigPrefix = (String) args[2];
		}
		System.out.println("Prefix: " + wsigPrefix);
		if (wsigPrefix != null && !wsigPrefix.equals("")) {
			sd.addProperties(new Property(WSIG_PREFIX, wsigPrefix));
		}

		dfad.addServices(sd);

		// DF registration
		try {
			DFService.register(this, dfad);
		} catch (Exception e) {
			System.out.println("Problem during DF registration: ");
			e.printStackTrace();
			;
			doDelete();
		}

		startDate = new Date();

		// Add Reconfigure behaviour
		

		// try {
		// hal = new HardwareAbstractionLayer(this);
		// } catch (KnowledgeException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// } catch (BlackboardUpdateException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
		register();
		// addBehaviour(new ReconfigureBehaviour(null, this));
		reconf = new ReconfigureBehaviour(hal, this);
		System.out.println("Added behaviour");
		System.out.println("HAL created");

		addBehaviour(new CyclicBehaviour() {
			private static final long serialVersionUID = 6925459044662459048L;

			public void action() {
				 
				ACLMessage msg = myAgent.receive(template);
				if (msg != null) {
					Action actExpr = null;
					System.out.println(msg.getSender() + msg.getContent());
					try {
						System.out.println("RECEIVED message: " + msg.toString());
						actExpr = (Action) getContentManager()
								.extractContent(msg);
						AgentAction action = (AgentAction) actExpr.getAction();
						
						//if (action instanceof ReconfigureBehaviour) {
						if(msg.getPerformative() == (ACLMessage.REQUEST)) {	
						System.out.println("TEST RECONFIGUREAGENT");
							serveReconfigureGetModules(
									(ReconfigureBehaviour) action, actExpr, msg);
						}
							//}
					} catch (Exception e) {
						e.printStackTrace();
					} //ACLMessage
					msg = receive();
					if (!msg.getSender().equals(this.getAgent().getAID())) {
						if (msg.getPerformative() == MessageType.CAN_EXECUTE_PRODUCT_STEP) {
							String result = canExecute(msg.getContent());
							System.out.println(result);
							if (!result.equals("false")) {
								ACLMessage reply = msg.createReply();
								reply.setPerformative(MessageType.AVAILABLE_TO_PLAN);
								reply.setContent(result);
								send(reply);
							}
						}
						if (msg.getPerformative() == MessageType.PLAN_PRODUCT_STEP) {
							String result = schedule(msg.getContent());
							ACLMessage reply = msg.createReply();
							reply.setPerformative(MessageType.CONFIRM_PLANNED);
							reply.setContent(result);
							send(reply);
						}
						if (msg.getPerformative() == MessageType.PULSE_UPDATE) {

							// This is the update information for the WIMP!
							JsonObject equipletUpdate = new JsonObject();
							equipletUpdate.addProperty("receiver", "interface");
							equipletUpdate.addProperty("subject",
									"update_equiplet");
							equipletUpdate.addProperty("id", EQName);
							equipletUpdate.addProperty("services", serverLists);
							JsonObject status = new JsonObject();
							status.addProperty("type", "success");
							status.addProperty("content", "NORMAL");
							equipletUpdate.add("status", status);

							JsonObject mode = new JsonObject();
							mode.addProperty("type", "success");
							mode.addProperty("content", "NORMAL");
							equipletUpdate.add("mode", mode);

							JsonObject equipletDetails = new JsonObject();
							equipletDetails.addProperty("status",
									equipletActive);
							equipletDetails.addProperty("plannedSteps",
									productStepCounter);
							equipletDetails.addProperty("successfulSteps",
									productStepSuccesCounter);
							equipletDetails.addProperty("failedSteps",
									productStepFailedCounter);

							equipletUpdate.add("details", equipletDetails);

							ACLMessage reply = msg.createReply();
							reply.setPerformative(MessageType.PULSE_UPDATE);
							reply.setContent(equipletUpdate.toString());
							send(reply);
						}
					}

				}
				block();
				
					

				
						
					}

				
		});
	}

	public void deregister() {
		System.out.println("deregister equipletAgent");
		try {
			DFService.deregister(this);
		} catch (FIPAException e) {
			e.printStackTrace();
		}
	}

	public void register() {

		serverLists = "";
		// serviceList.clear();
		// try {
		// serviceList = hal.getSupportedServices();
		// } catch (KnowledgeException e) {
		// e.printStackTrace();
		// } catch (Exception e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
		DFAgentDescription dfd = new DFAgentDescription();
		// for (int i = 0; i < serviceList.size(); i++) {
		// ServiceDescription sd = new ServiceDescription();
		// sd.setName(serviceList.get(i).getName());
		// sd.setType(serviceList.get(i).getName());
		// dfd.addServices(sd);
		// serverLists += serviceList.get(i).getName() + ",";
		// }
		// try {
		// DFService.register(this, dfd);
		// } catch (FIPAException e) {
		// e.printStackTrace();
		// }
	}

	private void serveReconfigureGetModules(ReconfigureBehaviour abs,
			Action actExpr, ACLMessage msg) {
		System.out.println("ReconfigureAgent.serveReconfigureGetModules");
		
		jade.util.leap.ArrayList result = new jade.util.leap.ArrayList();
		try {
			
			result.fromList(reconf.getModules());
		} catch (FactoryException | JarFileLoaderException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("ReconfigureAgent.serveReconfigureGetModules sending reply");
		sendNotification(actExpr, msg, ACLMessage.INFORM, result);
	}

	private void sendNotification(Action actExpr, ACLMessage request,
			int performative, Object result) {
		// Send back a proper reply to the requester
		ACLMessage reply = request.createReply();
		if (performative == ACLMessage.INFORM) {
			reply.setPerformative(ACLMessage.INFORM);
			try {
				ContentElement ce = null;
				if (result != null) {
					// If the result is a java.util.List, convert it into a
					// jade.util.leap.List t make the ontology "happy"
					if (result instanceof java.util.List) {
						jade.util.leap.ArrayList l = new jade.util.leap.ArrayList();
						l.fromList((java.util.List) result);
						result = l;
					}
					ce = new Result(actExpr, result);
				} else {
					ce = new Done(actExpr);
				}
				getContentManager().fillContent(reply, ce);
			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			reply.setPerformative(performative);

		}
		reply.addUserDefinedParameter(ACLMessage.IGNORE_FAILURE, "true");
		send(reply);
	}

	/**
	 * canExecute() This function checks if the requested service can be
	 * executed on this Equiplet.
	 * 
	 * @return false if the service can't be executed, returns an startTime,
	 *         duration and ID of the product step if it can be executed.
	 */
	private String canExecute(String msg) {
		System.out.println("Can execute?");
		JsonObject productSteps = new JsonParser().parse(msg).getAsJsonObject();
		for (int i = 0; i < serviceList.size(); i++) {
			if (serviceList.get(i).getName()
					.equals(productSteps.get("service").getAsString())) {

				JsonObject message = new JsonObject();
				message.addProperty("startTime", "0");
				message.addProperty("duration", "100");
				message.addProperty("productStepId", productSteps.get("id")
						.getAsString());

				return message.toString();
			}
		}
		return "false";
	}

	/**
	 * schedule() This function schedules a job for the equiplet and places it
	 * in the equipletSchedule arraylist.
	 * 
	 * @return true if the planning has been done succesfull.
	 */
	private String schedule(String msg) {
		System.out.println("Can Schedule?");

		JsonObject productSteps = new JsonParser().parse(msg).getAsJsonObject();
		Job job = new Job(productSteps.get("startTime").getAsString(),
				productSteps.get("duration").getAsString(), productSteps.get(
						"productStepId").getAsString(), productSteps.get(
						"productStep").getAsJsonObject());
		equipletSchedule.add(job);
		productStepCounter++;
		System.out.println(equipletSchedule.size() + " =Length");
		if (scheduleCounter == 0) {
			executeProductStep();
		}
		return "true";
	}

	/**
	 * takeDown() This function is being called on when the Agent is being
	 * terminated. The EquipletAgent deregisters himself at the DF so that
	 * ProductAgents wont find him anymore.
	 */
	@Override
	protected void takeDown() {
		try {
			DFService.deregister(this);
			getContainerController().kill();
		} catch (Exception e) {
		}
	}

	/**
	 * executeProductStep() This function lets the HAL executes a product step.
	 * The onTranslationFinished function is being called upon when this process
	 * has finished.
	 */
	private void executeProductStep() {
		if (scheduleCounter < equipletSchedule.size()) {
			equipletActive = true;
			System.out.println("Execute PS");
			hal.translateProductStep(equipletSchedule.get(scheduleCounter)
					.getProductStep());
			scheduleCounter++;
		} else {
			equipletActive = false;
			scheduleCounter = 0;
			equipletSchedule.clear();
		}
	}

	@Override
	public void onProcessStatusChanged(String status, Module module,
			HardwareStep hardwareStep) {
		if (status.equals("FAILED")) {
			equipletActive = false;
			scheduleCounter = 0;
			equipletSchedule.clear();
			// Notify Product that failed and remove from schedule.
			// Log that process execute failed.
			productStepFailedCounter++;
		}
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
	}

	@Override
	public void onTranslationFinished(ProductStep productStep,
			java.util.ArrayList<HardwareStep> hardwareStep) {
		System.out.println("Translating finished, Hardwarestep created");
		System.out.println(hardwareStep.size());
		hal.executeHardwareSteps(hardwareStep);
	}

	@Override
	public void onTranslationFailed(ProductStep productStep) {
	}

	@Override
	public void onExecutionFinished() {
		System.out.println("Hardware Step Executed");
		productStepSuccesCounter++;
		executeProductStep();
	}

	@Override
	public String getEquipletName() {
		// TODO Auto-generated method stub
		return EQName;
	}

	@Override
	public void onEquipletStateChanged(String state) {
		// TODO Auto-generated method stub
		machineState = state;
	}

	@Override
	public void onEquipletModeChanged(String mode) {
		// TODO Auto-generated method stub
		machineMode = mode;
	}

	@Override
	public void onExecutionFailed() {
		// TODO Auto-generated method stub

	}

}
