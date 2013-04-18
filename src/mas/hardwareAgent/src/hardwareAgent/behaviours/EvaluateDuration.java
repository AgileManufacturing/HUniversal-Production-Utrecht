package hardwareAgent.behaviours;

/**
 * @file EvaluateDuration.java
 * @brief Handles the GetServiceStepDuratation Message.
 * @date Created: 12-04-13
 *
 * @author Thierry Gerritse  
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

import java.sql.ResultSet;
import java.sql.SQLException;

import hardwareAgent.EquipletStepMessage;
import hardwareAgent.HardwareAgent;
import hardwareAgent.Module;
import hardwareAgent.TimeData;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import rexos.libraries.knowledge.KnowledgeDBClient;
import rexos.libraries.knowledge.Queries;
import rexos.libraries.knowledge.Row;
import serviceAgent.ServiceStepMessage;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

import newDataClasses.*;

public class EvaluateDuration extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetServiceStepDuration");
	private HardwareAgent hardwareAgent;
	
	public EvaluateDuration(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
	}

	@Override
	public void handle(ACLMessage message) {
		Object contentObject = null;
		String contentString = message.getContent();

		try {
			contentObject = message.getContentObject();
		} catch (UnreadableException e) {
			// System.out.println("Exception Caught, No Content Object Given");
		}
		System.out.format("%s received message from %s (%s:%s)%n", myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(), contentObject == null ? contentString : contentObject);

		try {
			ObjectId objectId = null;
			BasicDBObject serviceStep = null;
			try {
				objectId = (ObjectId) message.getContentObject();
				serviceStep = (BasicDBObject) hardwareAgent.getServiceStepsBBClient().findDocumentById(objectId);
			} catch (UnreadableException | InvalidDBNamespaceException e) {
				e.printStackTrace();
				myAgent.doDelete();
			}
			
			long stepType = serviceStep.getLong("type");
			BasicDBObject parameters = (BasicDBObject) serviceStep.get("parameters");
			
			String serviceName = serviceStep.getString("serviceName");
						
			/**
			 * haal de naam van de leidende module uit knowledge db aan de hand van de servicestep.service (ofzo)
			 * 
			 * String moduleName = iets vanuit de modulefactory geloof ik?
			 * 
			 */
			
			Module module = hardwareAgent.GetModuleByName("gripper");		
			
			EquipletStepMessage[] equipletSteps = module.getEquipletSteps(parameters);
			
			long stepDuration = 0l;
			
			for(EquipletStepMessage equipletStep : equipletSteps){
				
				TimeData td = equipletStep.timeData;
				stepDuration += td.getDuration();
				
			}
			
			ScheduleData schedule = new ScheduleData();
			schedule.fillWithBasicDBObject(((BasicDBObject) serviceStep.get("scheduleData")));
			schedule.setDuration(stepDuration);

			hardwareAgent.getServiceStepsBBClient().updateDocuments(
					new BasicDBObject("_id", objectId), 
					new BasicDBObject("$set", schedule.toBasicDBObject()));
			// plaats equipletsteps en hun duration en zijn status op eveluating
			// op bb,

			ACLMessage reply;
			reply = message.createReply();
			reply.setContentObject(objectId);
			reply.setOntology("GetServiceStepDurationResponse");
			myAgent.send(reply);

			// zet duration van de betreffende service step

			// stuur peter een reactie met het staat er
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
