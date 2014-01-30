/**
 * @file src/REXOS/MAS/agents/equiplet_agent/behaviours/ScheduleInformation.java
 * @brief Behaviour for giving information about the current schedule of this equiplet.
 * 			Can also set the lock on demand on the schedule of this equiplet 
 * @date Created: 04 nov. 2013
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
package agents.equiplet_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;
import java.util.UUID;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.schedule.data_classes.EquipletScheduleInformation;
import libraries.schedule.data_classes.ScheduleException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

public class ScheduleInformation extends ReceiveBehaviour {

	/**
	 * @var static final long serialVersionUID 
	 * 		The serial version UID for this class
	 */
	private static final long serialVersionUID = -1997392331052927657L;

	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("ScheduleInformation");

	private EquipletAgent equipletAgent;


	public ScheduleInformation(EquipletAgent equipletAgent) {
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent = equipletAgent;
	}

	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				ACLMessage response = message.createReply();
				
				if(message.getContent() == "lock") {
					UUID scheduleKey = equipletAgent.getSchedule().getScheduleLock();
					
					if(scheduleKey != null){
						//send agree message according to the FIPA standard
						response.setPerformative(ACLMessage.AGREE);
						response.setContentObject(scheduleKey);
						equipletAgent.send(response);
						
						EquipletScheduleInformation freeTimeData =  equipletAgent.getSchedule().getFreeTimeSlots(null, null);
						
						
						// get schedule and calculate load
						
						//message the questioning agent with the equiplet's schedule
						response = message.createReply();
						response.setPerformative(ACLMessage.INFORM);
						response.setContentObject(freeTimeData);
					}				
					// schedule is locked, send refuse message
					else {
						response.setPerformative(ACLMessage.REFUSE);
					}
				} else {
					// Don't lock the the schedule, just get and send it
					response.setPerformative(ACLMessage.INFORM);
					
					EquipletScheduleInformation freeTimeData =  equipletAgent.getSchedule().getFreeTimeSlots(null, null);
					
					response.setContentObject(freeTimeData);
				}
				
				// Send the response
				equipletAgent.send(response);
			} catch (IOException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
			} catch (ScheduleException e) {
				ACLMessage response = message.createReply();
				response.setPerformative(ACLMessage.FAILURE);
				equipletAgent.send(response);
			}

		}
	}
}
