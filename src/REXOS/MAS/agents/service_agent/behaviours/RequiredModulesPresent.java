/**
 * @file rexos/mas/service_agent/behaviours/RequiredModulesPresent.java
 * @brief Handles the RequiredModulesPresent message from the hardwareAgent that indicates whether all required modules
 *        for a service are present.
 * @date Created: 10 oct. 2013
 * 
 * @author Roy Scheefhals
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package agents.service_agent.behaviours;

import java.io.IOException;
import java.io.Serializable;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.ParentBehaviourCallback;
import agents.service_agent.ServiceAgent;
import agents.shared_behaviours.ReceiveBehaviour;

/**
 * This behaviour handles the CheckForModulesResponse message. The hardwareAgent sends this message in response to the
 * CheckForModules message to indicate whether all required modules for a service are present. It will generate a
 * timeout after a specified period if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class RequiredModulesPresent extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -7514590874724120963L;
	
	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("RequiredModulesPresent");
	
	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * @var ParentBehaviourCallback parentBehaviourCallback
	 * 		The parentbehaviour callback this redirect calls back to
	 */
	private ParentBehaviourCallback parentBehaviourCallback;
	
	/**
	 * @var int[] moduleGroupIds
	 * 		The ModuleGroup Ids used to check required modules
	 */
	private int[] moduleGroupIds;
	
	/**
	 * @var String conversationId
	 * 		The conversationId used for the message
	 */
	private String conversationId;

	/**
	 * Creates a new RequiredModulesPresentfffvc instance with the specified parameters.
	 * 
	 * @param serviceAgent the agent this behaviour belongs to.
	 * @param parentBehaviourCallback the parentbehaviour this behaviour calls back to 
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 * @param moduleGroupIds The ModuleGroup Ids used to check required modules
	 */
	public RequiredModulesPresent(ServiceAgent serviceAgent, ParentBehaviourCallback parentBehaviourCallback,
			String conversationId, int[] moduleGroupIds) {
		super(serviceAgent, MESSAGE_TEMPLATE);
		this.serviceAgent = serviceAgent;
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.moduleGroupIds = moduleGroupIds;
		this.conversationId = conversationId;
	}

	@Override
	public void onStart(){
		ACLMessage queryIFMessage = new ACLMessage(ACLMessage.QUERY_IF);
		queryIFMessage.setConversationId(conversationId);
		queryIFMessage.addReceiver(serviceAgent.getHardwareAgentAID());
		queryIFMessage.setOntology("RequiredModulesPresent");
		if (moduleGroupIds != null){
			try {
				queryIFMessage.setContentObject(moduleGroupIds);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, "", e);
			}
		}
		serviceAgent.send(queryIFMessage);
	
	}
	/**
	 * Handles an incoming message from the hardwareAgent. Doens't do any logic and will callback to the parentbehaviour.
	 * If a timeout occurs this method will call doDelete() on the service agent.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			parentBehaviourCallback.callback(message, null);
			serviceAgent.removeBehaviour(this);
		} else {
			Logger.log(LogLevel.WARNING, "" + serviceAgent.getName() + " - CheckForModulesResponse timeout!");
			serviceAgent.doDelete();
		}
	}
}
