package rexos.mas.equiplet_agent;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.util.Timer;
import java.util.TimerTask;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;

import com.mongodb.BasicDBObject;

public class NextProductStepTimer extends Timer{
	EquipletAgent equipletAgent;
	
	/**
	 * @var int nextUsedTimeSlot
	 * The next used time slot.
	 */
	private long nextUsedTimeSlot;

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
	public void setNextUsedTimeSlot(long nextUsedTimeSlot){
		this.nextUsedTimeSlot = nextUsedTimeSlot;
		cancel();
		if(nextUsedTimeSlot != -1){
			long startTimeSlot = nextUsedTimeSlot * timeSlotLength + firstTimeSlot;
			long currentTime = System.currentTimeMillis();
			this.schedule(new NextProductStepTask(), startTimeSlot - currentTime);
		}
	}
	
	/**
	 * Getter for getting the nextUsedTimeSlot.
	 * @return the next used timeslot
	 */
	public long getNextUsedTimeSlot() {
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
				e.printStackTrace();
			}
		}
	}
}