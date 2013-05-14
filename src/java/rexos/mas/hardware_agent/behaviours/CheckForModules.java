package rexos.mas.hardware_agent.behaviours;

/**
 * @file CheckForModules.java
 * @brief Handles the CheckForModules message
 * @date Created: 12-04-13
 *
 * @author Thierry Gerritse
 * @author Jan-Willem Willebrands
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
 * 
 * @note 2013-05-14 JWW: This should be adapted to use MOST once it's done.
 **/

import java.util.ArrayList;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.hardware_agent.HardwareAgent;

public class CheckForModules extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 * SerialUID for this class.
	 **/
	private static final long serialVersionUID = 1L;
	
	private static MessageTemplate messageTemplate = MessageTemplate
			.MatchOntology("CheckForModules");
	private HardwareAgent hardwareAgent;

	/**
	 * 
	 * Instantiates a new check for module.
	 * 
	 */
	public CheckForModules(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
	}
	
	/**
	 * Returns the available modules for this equiplet. This data is currently retrieved from the
	 * knowledge base. Once MOST is implemented, this method should take MOST data into account.
	 * 
	 * @return An arraylist containing the ids of the available modules for this equiplet.
	 *
	 **/
	private ArrayList<Integer> getAvailableModules() {
		ArrayList<Integer> availableModules = new ArrayList<Integer>();
		try {
			
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] rows = client.executeSelectQuery(
					Queries.MODULES_PER_EQUIPLET, hardwareAgent
							.getEquipletAgentAID().getLocalName());
			for (Row row : rows) {
				availableModules.add((Integer) row.get("module"));
			}
		} catch (KeyNotFoundException | KnowledgeException ex) {
			// Return the current (possibly empty) arraylist if reading 
			// from the knowledge db fails for whatever reason.
		}
		
		return availableModules;
	}

	/**
	 * Responds to incoming messages querying whether a set of modules is available.
	 * This method will respond with a CheckForModulesResponse message.
	 * If all modules are available, a CONFIRM will be sent.
	 * If one or more modules are missing, a DISCONFIRM will be sent.
	 * @param ACLMessage The incoming message.
	 */
	@Override
	public void handle(ACLMessage message) {
		boolean modulesPresent = true;
		
		try {
			int[] moduleIds = (int[]) message.getContentObject();
			ArrayList<Integer> availableModules = getAvailableModules();
			

			for (int moduleId : moduleIds) {
				if (!availableModules.contains(moduleId)) {
					modulesPresent = false;
					break;
				}
			}
		} catch (UnreadableException ex) {
			modulesPresent = false;
		} finally {
			ACLMessage reply;
			reply = message.createReply();
			reply.setOntology("CheckForModulesResponse");
			if (modulesPresent) {
				reply.setPerformative(ACLMessage.CONFIRM);
			} else {
				reply.setPerformative(ACLMessage.DISCONFIRM);
			}
			
			myAgent.send(reply);
		}

	}
}
