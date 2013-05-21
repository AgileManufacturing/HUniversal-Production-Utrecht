package rexos.mas.equiplet_agent.behaviours;

/**
 * @file AbortStep.java
 * @brief responds to abort step message.
 * @date Created: 2013-05-17
 * 
 * @author Thierry Gerritse
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import org.bson.types.ObjectId;
import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ProductionStep;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.hardware_agent.EquipletStepMessage;
import com.mongodb.BasicDBObject;

public class AbortStep extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate The messageTemplate this behaviour
	 *      listens to. This behaviour listens to the ontology: AbortStep.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate
			.MatchOntology("AbortStep");

	/**
	 * @var EquipletAgent equipletAgent The equipletAgent related to this
	 *      behaviour.
	 */
	private EquipletAgent equipletAgent;
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new can perform step.
	 * 
	 * @param a The agent for this behaviour
	 */
	public AbortStep(Agent a, BlackboardClient equipletBBClient) {
		super(a, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.equipletBBClient = equipletBBClient;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the
	 * response to the AbortStep checks the step status, and aborts it if able to.
	 * else send back failure
	 * @param message The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		Logger.log("%s received message from %s%n", myAgent.getLocalName(), message
				.getSender().getLocalName(), message.getOntology());

		ProductionStep productStep = null;
		try {
			// gets the productstep out of the message.
			ObjectId productStepEntryId = null;
			
			productStepEntryId = equipletAgent.getRelatedObjectId(message.getConversationId());

			BasicDBObject step = (BasicDBObject)equipletBBClient.findDocumentById(productStepEntryId);
			EquipletStepMessage esm = new EquipletStepMessage(step);
			
			if(esm.getStatus()==StepStatusCode.PLANNED){
				
				equipletBBClient.updateDocuments(new BasicDBObject("_id",productStepEntryId), new BasicDBObject("$set",new BasicDBObject("status",StepStatusCode.ABORTED)));
				
			}
			else{
				
			ACLMessage reply = message.createReply();
			
			reply.setPerformative(ACLMessage.FAILURE);
			 
			myAgent.send(reply);
				
			}
			
		}catch(GeneralMongoException | InvalidDBNamespaceException e){
			
		}
	}
	
}
