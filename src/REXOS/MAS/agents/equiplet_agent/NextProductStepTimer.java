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
package agents.equiplet_agent;

import jade.lang.acl.ACLMessage;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.ProductStep;
import agents.data_classes.ScheduleData;
import agents.data_classes.StepStatusCode;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

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
	 * @var long nextUsedTimeSlot
	 *      The next used time slot.
	 */
	private long nextUsedTimeSlot;

	/**
	 * @var long firstTimeSlot
	 *      The first time slot of the grid.
	 */
	private long firstTimeSlot;

	/**
	 * @var long timeSlotLength
	 *      The length of a time slot in milliseconds.
	 */
	private long timeSlotLength;

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
		nextUsedTimeSlot = -1;
	}

	/**
	 * Function for setting the next used timeslot.
	 * 
	 * @param nextUsedTimeSlot the index of the next used timeslot, fill 0 when not used.
	 */
	public void setNextUsedTimeSlot(long nextUsedTimeSlot) {
		this.nextUsedTimeSlot = nextUsedTimeSlot;
		if(task != null) {
			task.cancel();
		}
		if(nextUsedTimeSlot != -1) {
			long startPlannedTimeSlot = (nextUsedTimeSlot * timeSlotLength) + firstTimeSlot;
			long currentTime = System.currentTimeMillis();
			task = new NextProductStepTask();
			Logger.log(LogLevel.DEBUG, "%d Equiplet agent - trying to schedule: %d (%d - %d)%n", EquipletAgent.getCurrentTimeSlot(), (startPlannedTimeSlot - currentTime),
					startPlannedTimeSlot, currentTime);
			if(startPlannedTimeSlot > currentTime) {
				schedule(task, startPlannedTimeSlot - currentTime);
				Logger.log(LogLevel.DEBUG, "Equiplet agent - schedule set to: %d (%d - %d)%n", (startPlannedTimeSlot - currentTime),
						startPlannedTimeSlot, currentTime);
			} else {
				Logger.log(LogLevel.ERROR, "Equiplet agent - timer startPlannedTimeSlot is in the past " + startPlannedTimeSlot);
			}
		}
	}

	/**
	 * Getter for getting the nextUsedTimeSlot.
	 * 
	 * @return the next used timeslot
	 */
	public long getNextUsedTimeSlot() {
		return nextUsedTimeSlot;
	}

	/**
	 * Function for getting the timeSlotLength
	 * 
	 * @return the timeSlotLength
	 */
	public long getTimeSlotLength() {
		return timeSlotLength;
	}

	/**
	 * Function for setting the timeSlotLength
	 * 
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
	 * 
	 * when this function is called it reschedules the timer to the next step
	 * 
	 */
	public void rescheduleTimer() {
		try {
			BasicDBObject query = new BasicDBObject("status", StepStatusCode.PLANNED.name());
			BasicDBObject orderby = new BasicDBObject("scheduleData", new BasicDBObject("startTime", "1"));
			BasicDBObject findquery = new BasicDBObject("$query", query).append("$orderby", orderby);
			List<DBObject> objects = equipletAgent.getProductStepBBClient().findDocuments(findquery);
			if(!objects.isEmpty()) {
				ProductStep nextProductStep = new ProductStep((BasicDBObject) objects.get(0));
				equipletAgent.setNextProductStep(nextProductStep.getId());
				ScheduleData scheduleData = nextProductStep.getScheduleData();
				if(nextUsedTimeSlot == -1 || scheduleData.getStartTime() < nextUsedTimeSlot) {
					setNextUsedTimeSlot(scheduleData.getStartTime());
				} else {
					Logger.log(LogLevel.DEBUG, "%d Equiplet agent - Earliest step is not before current step (%d)%n", EquipletAgent.getCurrentTimeSlot(), scheduleData.getStartTime());
				}
			} else {
				Logger.log(LogLevel.DEBUG, "%d Equiplet agent - no more steps on PLANNED%n", EquipletAgent.getCurrentTimeSlot());
				setNextUsedTimeSlot(-1);
			}
		} catch(GeneralMongoException | InvalidDBNamespaceException e) {
			Logger.log(LogLevel.ERROR, "MongoDb failed at " + this.equipletAgent.getAID().getLocalName(), e);
		}

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
				ProductStep productStep =
						new ProductStep((BasicDBObject) equipletAgent.getProductStepBBClient().findDocumentById(productStepEntry));

				Logger.log(LogLevel.DEBUG, "%d Equiplet agent - Asking PA to start with step at time (%d)%n", EquipletAgent.getCurrentTimeSlot(), productStep.getScheduleData().getStartTime());
				
				// ask the productAgent to start the production of the step.
//				ACLMessage answer = new ACLMessage(ACLMessage.QUERY_IF);
//				answer.setConversationId(conversationId);
//				answer.addReceiver(productStep.getProductAgentId());
//				answer.setOntology("StartStepQuestion");
//				equipletAgent.send(answer);

				nextUsedTimeSlot = -1;
				
//				 TODO: delete below after testing
				 ACLMessage test = new ACLMessage(ACLMessage.QUERY_IF);
				 test.setConversationId(conversationId);
				 test.addReceiver(equipletAgent.getAID());
				 test.setOntology("StartStep");
				 equipletAgent.send(test);
			} catch(InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
	}
}