/**
 * @file rexos/mas/equiplet_agent/NextProductStepTimer.java
 * @brief Timer for executing the next product step.
 * @date Created: 2013-04-02
 *
 * @author Hessel Meulenbeld
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
package rexos.mas.equiplet_agent;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.util.Timer;
import java.util.TimerTask;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;

import com.mongodb.BasicDBObject;

/**
 * Timer for executing the next product step.
 **/
public class NextProductStepTimer extends Timer{
	EquipletAgent equipletAgent;
	
	/**
	 * @var int nextUsedTimeSlot
	 * The next used time slot.
	 */
	private int nextUsedTimeSlot;

	/**
	 * @var int firstTimeSlot
	 * The first time slot of the grid.
	 */
	private int firstTimeSlot;
	
	/**
	 * @var int timeSlotLength
	 * The length of a time slot in milliseconds.
	 */
	private int timeSlotLength;
	
	/**
	 * Constructor for the next product step timer.
	 * @param firstTimeSlot the first time slot from the grid/equiplet.
	 * @param timeSlotLength the length of a time slot.
	 */
	public NextProductStepTimer(int firstTimeSlot, int timeSlotLength){
		super();
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;
	}
	
	/**
	 * Function for setting the next used time slot.
	 * @param nextUsedTimeSlot the index of the next used timeslot, fill -1 when not used.
	 */
	public void setNextUsedTimeSlot(int nextUsedTimeSlot){
		this.nextUsedTimeSlot = nextUsedTimeSlot;
		cancel();
		if(nextUsedTimeSlot != -1){
			int startTimeSlot = nextUsedTimeSlot * timeSlotLength + firstTimeSlot;
			int currentTime = (int)System.currentTimeMillis();
			this.schedule(new NextProductStepTask(), startTimeSlot - currentTime);
		}
	}
	
	/**
	 * Getter for getting the nextUsedTimeSlot.
	 * @return the next used timeslot
	 */
	public int getNextUsedTimeSlot() {
		return nextUsedTimeSlot;
	}
	
	private class NextProductStepTask extends TimerTask {
		public NextProductStepTask() {
		}

		/**
		 * run method for this TimerTask.
		 */
		@Override
		public void run() {
			try {
				//Gets all the objects needed to start the next step.
				ObjectId productStepEntry = equipletAgent.getNextProductStep();
				String conversationId = equipletAgent.getConversationId(productStepEntry);
				BasicDBObject productStep = (BasicDBObject)equipletAgent.getEquipletBBClient().findDocumentById(productStepEntry);
				AID productAgent = new AID((String)productStep.get("productAgentId"), AID.ISGUID);

				//ask the productAgent to start the production of the step.
				ACLMessage answer = new ACLMessage(ACLMessage.QUERY_IF);
				answer.setConversationId(conversationId);
				answer.addReceiver(productAgent);
				answer.setOntology("StartStepQuestion");
				equipletAgent.send(answer);
			} catch (InvalidDBNamespaceException | GeneralMongoException e) {
				// TODO Auto-generated catch block
				Logger.log(e);
			}
		}
	}
}