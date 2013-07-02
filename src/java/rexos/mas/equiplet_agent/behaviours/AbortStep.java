/**
 * @file rexos/mas/equiplet_agent/behaviours/AbortStep.java
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

package rexos.mas.equiplet_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ProductStep;
import rexos.mas.data.StepStatusCode;
import rexos.mas.equiplet_agent.EquipletAgent;

import com.mongodb.BasicDBObject;

/**
 * Handles incoming abort step messages from the product agent and updates the
 * blackboard accordingly.
 */
public class AbortStep extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 **/
	private static final long serialVersionUID = -9022585847666136289L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The messageTemplate this behaviour
	 *      listens to. This behaviour listens to the ontology: AbortStep.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("AbortStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this
	 *      behaviour.
	 */
	private EquipletAgent equipletAgent;

	/**
	 * Instantiates a new can perform step.
	 * 
	 * @param agent The agent for this behaviour
	 */
	public AbortStep(EquipletAgent agent) {
		super(agent, messageTemplate);
		equipletAgent = agent;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the
	 * response to the AbortStep checks the step status, and aborts it if able
	 * to.
	 * else send back failure.
	 * 
	 * @param message
	 *            The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			Logger.log("%s received message from %s(%s)%n", myAgent.getLocalName(), message.getSender().getLocalName(),
					message.getOntology());
	
			try {
				// gets the productstep out of the message.
				ObjectId productStepEntryId = equipletAgent.getRelatedObjectId(message.getConversationId());
	
				BasicDBObject step =
						(BasicDBObject) equipletAgent.getProductStepBBClient().findDocumentById(productStepEntryId);
				ProductStep productStep = new ProductStep(step);
	
				//if the status is planned reschedule the timer
				if(productStep.getStatus() == StepStatusCode.PLANNED) {
					equipletAgent.cancelProductStep(productStepEntryId, "productagent canceled");
					equipletAgent.getTimer().rescheduleTimer();
					
				} else {
					ACLMessage reply = message.createReply();
					reply.setPerformative(ACLMessage.FAILURE);
					myAgent.send(reply);
				}
			} catch(InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(e);
				equipletAgent.doDelete();
			}
		}
	}
}
