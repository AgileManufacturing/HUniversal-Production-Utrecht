/**
 * @file rexos/mas/equiplet_agent/NextProductStepTimer.java
 * @brief Timer for executing the next product step.
 * @date Created: 2013-04-02
 * 
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
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
public class NextProductStepTimer extends Timer {
	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent to which the timer belongs.
	 */
	EquipletAgent equipletAgent;

	/**
	 * @var int nextUsedTimeSlot
	 *      The next used time slot.
	 */
	private int nextUsedTimeSlot;

	/**
	 * @var long firstTimeSlot
	 *      The first time slot of the grid.
	 */
	private long firstTimeSlot;

	/**
	 * @var int timeSlotLength
	 *      The length of a time slot in milliseconds.
	 */
	private int timeSlotLength;

	/**
	 * @var NextProductStepTask task
	 *      The task for this timer
	 */
	private NextProductStepTask task;

	/**
	 * Constructor for the next product step timer.
	 * 
	 * @param firstTimeSlot
	 *            the first time slot from the grid/equiplet.
	 * @param timeSlotLength
	 *            the length of a time slot.
	 * @param agent
	 *            The equipletAgent to which the timer belongs.
	 */
	public NextProductStepTimer(long firstTimeSlot, int timeSlotLength, EquipletAgent agent) {
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;
		equipletAgent = agent;
	}

	/**
	 * Function for setting the next used timeslot.
	 * 
	 * @param nextUsedTimeSlot the index of the next used timeslot, fill -1 when not used.
	 */
	public void setNextUsedTimeSlot(int nextUsedTimeSlot) {
		this.nextUsedTimeSlot = nextUsedTimeSlot;
		if(task != null) {
			task.cancel();
		}
		if(nextUsedTimeSlot != -1) {
			long startTimeSlot = nextUsedTimeSlot * timeSlotLength + firstTimeSlot;
			long currentTime = System.currentTimeMillis();
			task = new NextProductStepTask();
			Logger.log("Equiplet agent - trying to schedule: %d (%d - %d)%n", (startTimeSlot - currentTime), startTimeSlot, currentTime);
			if(startTimeSlot - currentTime > 0) {
				schedule(task, startTimeSlot - currentTime);
				Logger.log("Equiplet agent - schedule set to: %d (%d - %d)%n", (startTimeSlot - currentTime), startTimeSlot, currentTime);
			} else {
				Logger.log("");
			}
		}
	}

	/**
	 * Getter for getting the nextUsedTimeSlot.
	 * 
	 * @return the next used timeslot
	 */
	public int getNextUsedTimeSlot() {
		return nextUsedTimeSlot;
	}

	/**
	 * @return the timeSlotLength
	 */
	public int getTimeSlotLength() {
		return timeSlotLength;
	}

	/**
	 * @param timeSlotLength the timeSlotLength to set
	 */
	public void setTimeSlotLength(int timeSlotLength) {
		this.timeSlotLength = timeSlotLength;
	}

	/**
	 * Getter for the first time slot
	 * 
	 * @return the first time slot
	 */
	public long getFirstTimeSlot() {
		return firstTimeSlot;
	}

	/**
	 * Task in charge of starting the next product step.
	 */
	private class NextProductStepTask extends TimerTask {
		public NextProductStepTask() {}

		/**
		 * Grabs the next product step from the blackboard and checks with the
		 * Product Agent whether or not execution may be started.
		 */
		@Override
		public void run() {
			try {
				// Gets all the objects needed to start the next step.
				ObjectId productStepEntry = equipletAgent.getNextProductStep();
				String conversationId = equipletAgent.getConversationId(productStepEntry);
				BasicDBObject productStep =
						(BasicDBObject) equipletAgent.getEquipletBBClient().findDocumentById(productStepEntry);
				AID productAgent = new AID((String) productStep.get("productAgentId"), AID.ISGUID);

				// ask the productAgent to start the production of the step.
				ACLMessage answer = new ACLMessage(ACLMessage.QUERY_IF);
				answer.setConversationId(conversationId);
				answer.addReceiver(productAgent);
				answer.setOntology("StartStepQuestion");
				equipletAgent.send(answer);

				// TODO: delete below after testing
				ACLMessage test = new ACLMessage(ACLMessage.QUERY_IF);
				test.setConversationId(conversationId);
				test.addReceiver(equipletAgent.getAID());
				test.setOntology("StartStep");
				equipletAgent.send(test);
			} catch(InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(e);
			}
		}
	}
}