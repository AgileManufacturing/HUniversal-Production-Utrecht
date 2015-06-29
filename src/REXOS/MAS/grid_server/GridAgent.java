/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	GridAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class/Agent is being used to add agents to the grid.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-06-06
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
package MAS.grid_server;

import generic.Criteria;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import MAS.equiplet.EquipletAgent;
import MAS.product.ProductAgent;
import MAS.product.ProductStep;
import MAS.simulation.config.Config;
import MAS.simulation.config.Configuration;
import MAS.simulation.config.Configuration.ConfigException;
import MAS.simulation.config.IConfig;
import MAS.util.MASConfiguration;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import SCADA.BasicAgentInfo;
import SCADA.SCADABasicListener;
import SCADA.SCADADetailedListener;
import SCADA.StochasticsTemp;

public class GridAgent extends Agent{
	private static final long serialVersionUID = -720095833750151495L;

	private long productCounter = 0;
	private ArrayList<AID> basicListeners;
	private ArrayList<BasicAgentInfo> agentInformation;

	public GridAgent() {
		productCounter = 0;
		basicListeners = new ArrayList<AID>();
		agentInformation = new ArrayList<BasicAgentInfo>();
	}

	/**
	 * Setup the GridAgent and spawn some agents
	 */
	@Override
	protected void setup() {
		spawnTrafficAgent();
		spawnSupplyAgent();
		spawnEquipletAgent("EQ0", "hal");
		// TODO No arguments are given
		spawnProductAgent("PA1", null );
		
		addBehaviour(new GridAgentListenerBehaviour(this));
	
		testGetAgents();
	}

	/**
	 * This isn't used by GridAgent anymore but might need to be added into GridAgentListenerBehaviour
	 * TODO check this
	 *
	 */
	class GridListenerBehaviour extends CyclicBehaviour {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			ACLMessage msg = blockingReceive();
			if (msg != null) {
				System.out.println(getLocalName() + ": received new request to spwan agent.");

				try {
					ContainerController cc = getContainerController();
					String name = "PA" + productCounter++;

					// parse configurations
					LinkedList<ProductStep> productSteps = parseConfigurationProductSteps(msg.getContent());

					for (ProductStep productStep : productSteps) {
						// replace the criteria in each productstep by the actual identifiers of crates and objects
						productStep.updateCriteria(fillProductCriteria(productStep.getCriteria()));
					}

					// TODO hard coded, need to come from arguments
					Position startPosition = new Position(0, 0);
					Tick deadline = new Tick().add(1000000);

					Object[] arguments = new Object[] { Parser.parseProductConfiguration(productSteps, startPosition, deadline) };
					AgentController ac = cc.createNewAgent(name, ProductAgent.class.getName(), arguments);
					ac.start();

				} catch (StaleProxyException e) {
					e.printStackTrace();
				} catch (JSONException e) {
					System.err.println(getLocalName() + ": failed to parse product configurations: " + e.getMessage());
				}
			}
		}
	};

	/**
	 * TODO replace with Parser.parseProductConfiguration!
	 * 
	 * @param source
	 * @return
	 * @throws JSONException
	 */
	private LinkedList<ProductStep> parseConfigurationProductSteps(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("productSteps")) {
			JSONArray jsonSteps = json.getJSONArray("productSteps");
			LinkedList<ProductStep> steps = new LinkedList<ProductStep>();
			for (int i = 0; i < jsonSteps.length(); i++) {
				JSONObject jsonStep = jsonSteps.getJSONObject(i);
				if (jsonStep.has("service") && jsonStep.has("criteria")) {
					JSONObject jsonCriteria = jsonStep
							.getJSONObject("criteria");

					if (jsonCriteria.has("subjects")
							&& jsonCriteria.has("target")) {
						JSONObject jsonTarget = jsonCriteria
								.getJSONObject("target");
						JSONObject jsonSubjects = jsonCriteria
								.getJSONObject("subjects");

						JSONObject criteria = new JSONObject();
						criteria.put(Criteria.TARGET, jsonTarget);
						criteria.put(Criteria.SUBJECTS, jsonSubjects);

						String service = jsonStep.getString("service");
						ProductStep step = new ProductStep(i, service, criteria);
						steps.add(step);
					} else {
						System.err
								.println("no target or subject in criteria of product step");
					}
				} else {
					System.err
							.println("no service or criteria in product step");
				}
			}
			return steps;
		} else {
			throw new JSONException("no product steps in argument message");
		}
	}

	/**
	 * This method makes a call to the SupplyAgent. It sends the criteria to it.
	 * In turn the SupplyAgent returns the criteria targets and subjects with
	 * actual targets and subjects (crate codes and coordinates).
	 */
	// TODO Only works with the pick and place actions (the supply agent has to
	// be completely rewritten in order to make it compatible)
	// TODO delete completely!!!!!!!!! (because it is wrong, ugly and it should
	// feels bad)
	private JSONObject fillProductCriteria(JSONObject criteria) {
		if (criteria.length() > 0) {
			AID supplyAgent = new AID(MASConfiguration.SUPPLY_AGENT,
					AID.ISLOCALNAME);

			ACLMessage message = new ACLMessage(ACLMessage.QUERY_REF);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.addReceiver(supplyAgent);
			message.setConversationId(Ontology.CONVERSATION_SUPPLY_REQUEST);
			message.setReplyWith(Ontology.CONVERSATION_SUPPLY_REQUEST
					+ System.currentTimeMillis());
			message.setContent(criteria.toString());
			send(message);

			System.out.println(getLocalName() + ": has sent message to "
					+ supplyAgent.getLocalName());
			ACLMessage received = blockingReceive();
			if (received != null) {
				try {
					return new JSONObject(received.getContent());
				} catch (JSONException e) {
					e.printStackTrace();
				}
			} else {
				System.out.println(getLocalName()
						+ " failed to contact the supply agent");
			}
		}
		return criteria;
	}

	/**
	 * Create a traffic manager agent in the grid The traffic agent requires a
	 * map of all the equiplet with the travel distances to each other (for now)
	 */
	private void spawnTrafficAgent() {
		Map<String, Position> equipletMap = new HashMap<String, Position>();
		try {
			TrafficManager trafficAgent = new TrafficManager(equipletMap);

			ContainerController cc = getContainerController();
			AgentController ac = cc.acceptNewAgent(
					MASConfiguration.TRAFFIC_AGENT, trafficAgent);
			ac.start();
		} catch (StaleProxyException e) {
			System.err.println(this.getLocalName()
					+ ": spawnTrafficAgent fails");
		}
	}

	/**
	 * Create a supply agent (for now)
	 */
	private void spawnSupplyAgent() {
		try {
			ContainerController cc = getContainerController();
			AgentController ac = cc.createNewAgent(
					MASConfiguration.SUPPLY_AGENT, SupplyAgent.class.getName(),
					new Object[] {});
			ac.start();
		} catch (StaleProxyException e) {
			System.err.println(this.getLocalName()
					+ ": failed to create supply agent");
		}
	}
	
	/**
	 * Create a product agent (for now)
	 */
	public void spawnProductAgent(String name, String inputArguments){
		try {
			ContainerController cc = getContainerController();
			if(inputArguments != null){
				Config con = Config.read();
				IConfig config;
					config = Configuration.read(con);
	
				System.out.println("Simulation: configuration: " + config);
	
				StochasticsTemp stochastics = new StochasticsTemp(config);
				// parse configurations
				//LinkedList<ProductStep> productSteps = parseConfigurationProductSteps(null);
				LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();
	
				for (ProductStep productStep : productSteps) {
					// replace the criteria in each productstep by the actual identifiers of crates and objects
					productStep.updateCriteria(fillProductCriteria(productStep.getCriteria()));
				}

				// TODO hard coded, need to come from arguments
				Position startPosition = new Position(0, 0);
				Tick deadline = new Tick().add(1000000);
	
				Object[] arguments = new Object[] { Parser.parseProductConfiguration(productSteps, startPosition, deadline) };
				AgentController ac = cc.createNewAgent(name, ProductAgent.class.getName(), arguments);
				ac.start();
			}else{
				AgentController ac = cc.createNewAgent(name, ProductAgent.class.getName(), null);
				ac.start();
			}
				

		} catch (StaleProxyException e) {
			e.printStackTrace();
		}catch (JSONException e) {
			System.err.println(getLocalName() + ": failed to parse product configurations: " + e.getMessage());
			
		} catch (ConfigException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		
		}
		
	}


	/**
	 * Take this agent down
	 */
	@Override
	protected void takeDown() {
		System.out.println(getLocalName() + ": terminated");

	}

	public void testGetAgents() {
		DFAgentDescription description = new DFAgentDescription();
		SearchConstraints sc = new SearchConstraints();
		try {
			DFAgentDescription listOfAgents[] = DFService.search(this,
					description, sc);
			for (int i = 0; i < listOfAgents.length; i++) {
				System.out.println(listOfAgents[i]);
			}
		} catch (FIPAException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Create a Equiplet agent (for now)
	 * 
	 * @author Kevin Bosman
	 */
	public void spawnEquipletAgent(String name, String inputArguments) {
		try {
			ContainerController cc = getContainerController();
			// TODO check for other inputArguments instead of a string
			Object[] arguments;
			if(inputArguments != null){
				arguments = new Object[] { inputArguments };
			}else{
				arguments = new Object[] { "hal" };
			}
			AgentController ac = cc.createNewAgent(name, EquipletAgent.class.getName(), arguments);
			ac.start();
		} catch (StaleProxyException e) {
			System.err.println(this.getLocalName() + ": failed to create Equiplet agent");
			e.printStackTrace();
		}
	}

	/**
	 * Add agent as listener for update messages from the GridAgent
	 * 
	 * @param agent AID of agent which will be added as listener
	 */
	public void addBasicListener(AID agent) {
		if(!basicListeners.contains(agent)){
			basicListeners.add(agent);
		}
	}

	/**
	 * Send information to all listeners.
	 * 
	 * @param message  message containing information about the agent
	 */
	public void sendAgentInfo(String message){
		for (int i = 0; i < basicListeners.size(); i++) {
			sendUpdateMessage(basicListeners.get(i), message);
		}
	}
	
	/**
	 * Whenever an agent is killed, GridAgents needs to update clients and the list
	 * 
	 * @param content information about the agent which is taken down
	 */
	public void onAgentTakeDown(String content) {
		System.err.println("onAgentTakeDown");
		// Inform all listeners about take down off the agent in the grid
		for (int i = 0; i < basicListeners.size(); i++) {
			ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
			msg.addReceiver(basicListeners.get(i));
			msg.setConversationId(Ontology.CONVERSATION_AGENT_TAKEDOWN);
			msg.setContent(content);
			send(msg);
		}
		String agentName = "";
		try {
			JSONObject object = new JSONObject(content);
			agentName = object.getJSONObject("agent").getString("id");
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.err.println(agentName);
		// Search for the agent in agentInformation
		if(agentName != null){
			for(int index = 0; index < agentInformation.size(); index++){
				if(agentInformation.get(index).getAID().getLocalName().equals(agentName)){
					// Remove the agent from the agentInformation arraylist
					agentInformation.remove(index);
					break;
				}
			}
		}
	}
	
	
	/**
	 * Send update message to the agent
	 * 
	 * TODO SCADA is this used?
	 * @param agent receiver
	 * @param update message containing the update
	 */
	private void sendUpdateMessage(AID agent, String update) {
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);

		message.addReceiver(agent);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		message.setContent(update);
		send(message);
	}
	
	/** 
	 * Add new basic agent information to agentInformation arraylist 
	 * 
	 * @param bai BasicAgentInfo
	 */
	public void addBasicAgentInfo(BasicAgentInfo bai) {
		if(!agentInformation.contains(bai)) {
			agentInformation.add(bai);
		}
	}
	
	/** 
	 * Remove basic agent information from agentInformation arraylist 
	 * 
	 * @param bai BasicAgentInfo
	 */
	public void removeBasicAgentInfo(BasicAgentInfo bai) {
		if(agentInformation.contains(bai)) {
			agentInformation.remove(bai);
		}
	}
	
	/**
	 * Create a JSONObject containing all basicInformation about the agents in the grid 
	 * 
	 * @return JSONObject containing agentInformation
	 */
	public JSONObject getJSONOfOverview() {
		JSONObject object = new JSONObject();
		try {
			object.put("command", "GET_OVERVIEW");
			JSONArray array = new JSONArray();
			for(BasicAgentInfo bai : agentInformation) {
				array.put(bai.getJSONObject());
			}
			object.put("agents", array);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return object;
	}
	
	/**
	 * Update the state of an agent in agentInformation arraylist
	 * @param agent
	 * @param message
	 */
	public void updateStateInfo(AID agent, String message){
		String state = "";
		try {
			JSONObject object = new JSONObject(message);
			JSONObject a = new JSONObject();
			a = object.getJSONObject("agent");
			state = a.getString("state");
		} catch (JSONException e) { 
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		// update local agent info state
		for(int i = 0; i < agentInformation.size(); i++){
			if(agentInformation.get(i).getAID().equals(agent)){
				agentInformation.get(i).updateState(state);
				break;
			}
		}
	}
}
