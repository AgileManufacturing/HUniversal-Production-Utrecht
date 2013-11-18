/**
 * @file REXOS/MAS/agents/product_agent/behaviours/ScheduleInformation.java
 * @brief Behaviour for getting information about the current schedule of the given equiplets
 * 			Will also set the lock on the schedule of this equiplet
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
package agents.product_agent.behaviours;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

import agents.product_agent.ProductAgent;
import agents.shared_behaviours.ReceiveBehaviour;
import jade.core.AID;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import libraries.schedule.data_classes.EquipletScheduleInformation;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

public class ScheduleInformationBehaviour extends ReceiveBehaviour {

	/**
	 * @var long serialVersionUID
	 * 		The serialization UID of this class
	 */
	private static final long serialVersionUID = -7266342229623842111L;

	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("ScheduleInformation");

	private static final int TIMEOUT = 2000;

	private ProductAgent productAgent;
	private SchedulerBehaviour schedulerBehaviour;
	private AID[] equipletAgents;

	private HashMap<AID, EquipletScheduleInformation> equipletSchedules;
	private HashMap<AID, UUID> equipletKeys;
	private ArrayList<AID> refusedEquiplets;
	
	
	private int totalFinishedEquiplets;
	
	public ScheduleInformationBehaviour(ProductAgent productAgent, SchedulerBehaviour schedulerBehaviour, AID[] equipletAgents) {
		super(productAgent, TIMEOUT, MESSAGE_TEMPLATE);
		this.productAgent = productAgent;
		this.equipletAgents = equipletAgents;
		this.schedulerBehaviour = schedulerBehaviour;
		
		equipletSchedules = new HashMap<AID, EquipletScheduleInformation>();
		refusedEquiplets = new ArrayList<AID>();
		totalFinishedEquiplets = 0;
		
	}

	@Override
	public void onStart() {
		Logger.log(LogLevel.DEBUG, "Starting a new ScheduleInformationBehaviour");
		ACLMessage request = new ACLMessage(ACLMessage.QUERY_REF);
		request.setOntology("ScheduleInformation");
		//add the lock request to the message
		request.setContent("lock");
		for (AID equipletAgent : equipletAgents) {
			request.addReceiver(equipletAgent);
		}
		productAgent.send(request);
	}

	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				if (message.getPerformative() == ACLMessage.AGREE){
					//agree message has the key to the schedule lock, save it and pass to the schedulebehaviour
					UUID key = (UUID) message.getContentObject();
					equipletKeys.put(message.getSender(), key);
				
				} else if (message.getPerformative() == ACLMessage.REFUSE){
					//equiplet is already locked when refused, add to the refusedList
					refusedEquiplets.add(message.getSender());
					totalFinishedEquiplets ++;
				} else if (message.getPerformative() == ACLMessage.INFORM){
					//inform message contains the schedule data, add and check if all is dones
					totalFinishedEquiplets ++;
					EquipletScheduleInformation eqSchedule = (EquipletScheduleInformation) message.getContentObject();
					
					equipletSchedules.put(message.getSender(), eqSchedule);
					
					if (totalFinishedEquiplets == equipletAgents.length){ 
						// all equiplets have responded, 
						//callback to the scheduler with results and remove this behaviour
						schedulerBehaviour.callbackScheduleInformation(equipletSchedules, refusedEquiplets, schedulerBehaviour);
						productAgent.removeBehaviour(this);
					}
				}
			} catch (UnreadableException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		} else if (getTimeoutThrown()) {
			//we are missing responses from the rest of the equiplets
			//callback and remove the behaviour
			for ( AID equiplet : equipletAgents){
				if ( !equipletSchedules.keySet().contains(equiplet) && !refusedEquiplets.contains(equiplet)){
					refusedEquiplets.add(equiplet);
				}
			}
			schedulerBehaviour.callbackScheduleInformation(equipletSchedules, refusedEquiplets, schedulerBehaviour);
			productAgent.removeBehaviour(this);
		}

	}

}
