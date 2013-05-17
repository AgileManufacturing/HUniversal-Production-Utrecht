package rexos.mas.hardware_agent.behaviours;

/**
 * @file rexos/mas/hardware_agent/behaviours/EvaluateDuration.java
 * @brief Handles the GetServiceStepDuratation Message.
 * @date Created: 12-04-13
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
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ScheduleData;
import rexos.mas.hardware_agent.EquipletStepMessage;
import rexos.mas.hardware_agent.HardwareAgent;
import rexos.mas.hardware_agent.Module;
import rexos.mas.hardware_agent.ModuleFactory;
import rexos.mas.service_agent.ServiceStepMessage;

import com.mongodb.BasicDBObject;

/**
 * Class for the receivebehaviour for receiving messages with the ontology GetServiceStepDuration
 */
public class EvaluateDuration extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate
	 * The messageTemplate to match the messages.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetServiceStepDuration");
	
	/**
	 * @var HardwareAgent hardwareAgent
	 * The hardwareAgent of this behaviour.
	 */
	private HardwareAgent hardwareAgent;
	
	/**
	 * @var ModuleFactory moduleFactory
	 * The moduleFactory for this behaviour. 
	 */
	private ModuleFactory moduleFactory;

	/**
	 * Constructory
	 * @param a The agent
	 * @param moduleFactory The moduleFactory
	 */
	public EvaluateDuration(Agent a, ModuleFactory moduleFactory) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
		this.moduleFactory = moduleFactory;
	}

	/**
	 * @see ReceiveBehaviour#handle(ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		try {
			//get the serviceStepId
			ObjectId serviceStepId = (ObjectId) message.getContentObject();
			Logger.log("%s received message from %s (%s:%s)%n", myAgent.getLocalName(),
					message.getSender().getLocalName(), message.getOntology(), serviceStepId);
			//Evaluate the duration of the step
			EvaluateStepDuration(serviceStepId);

			//Send a message to the serviceAgent with the serviceStepId
			ACLMessage reply;
			reply = message.createReply();
			reply.setContentObject(serviceStepId);
			reply.setOntology("GetServiceStepDurationResponse");
			myAgent.send(reply);

		} catch (UnreadableException | IOException e) {
			e.printStackTrace();
			myAgent.doDelete();
		}
	}

	public void EvaluateStepDuration(ObjectId serviceStepId) {
		try {
			//get the serviceStep
			BasicDBObject dbServiceStep = (BasicDBObject) hardwareAgent.getServiceStepsBBClient().findDocumentById(serviceStepId);

			ServiceStepMessage serviceStep = new ServiceStepMessage(dbServiceStep);

			int stepDuration = 0;
			//get the leading module
			int leadingModule = hardwareAgent.getLeadingModule(serviceStep.getServiceId());
			Module module = moduleFactory.getModuleById(leadingModule);
			module.setConfiguration(hardwareAgent.getConfiguration());
			//create the equipletSteps
			EquipletStepMessage[] equipletSteps = 
					module.getEquipletSteps(serviceStep.getType(), serviceStep.getParameters());
			BlackboardClient equipletStepsBBClient = hardwareAgent.getEquipletStepsBBClient();
			ObjectId next = null;
			//calculate the duration and put the steps on the blackboard
			for (int i = equipletSteps.length - 1; i >= 0; i--) {
				EquipletStepMessage equipletStep = equipletSteps[i];
				stepDuration += equipletStep.getTimeData().getDuration();
				equipletStep.setServiceStepID(serviceStepId);
				equipletStep.setNextStepID(next);
				next = equipletStepsBBClient.insertDocument(equipletStep.toBasicDBObject());
			}
			//get the scheduleData and add the duration.
			ScheduleData schedule = serviceStep.getScheduleData();
			schedule.setDuration(stepDuration);

			//update the serviceStep
			hardwareAgent.getServiceStepsBBClient().updateDocuments(
					new BasicDBObject("_id", serviceStepId), new BasicDBObject("$set",
							new BasicDBObject("scheduleData", schedule.toBasicDBObject())));
			//if the serviceStep has an next step calculate the duration for that one too.
			if(serviceStep.getNextStep() != null){
				EvaluateStepDuration(serviceStep.getNextStep());
			}
		} catch (InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
			myAgent.doDelete();
		}
	}

}
