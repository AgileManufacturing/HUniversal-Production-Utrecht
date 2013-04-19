package equipletAgent;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.util.Timer;
import java.util.TimerTask;

import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class NextProductStepTimer extends Timer{
	EquipletAgent equipletAgent;
	
	/**
	 * @var long nextUsedTimeSlot
	 * The next used time slot.
	 */
	private long nextUsedTimeSlot;

	/**
	 * @var long firstTimeSlot
	 * The first time slot of the grid.
	 */
	private long firstTimeSlot;
	
	/**
	 * @var long timeSlotLength
	 * The length of a time slot.
	 */
	private long timeSlotLength;
	
	public NextProductStepTimer(long firstTimeSlot, long timeSlotLength){
		super();
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;
	}
	
	public void setNextUsedTimeSlot(long nextUsedTimeSlot){
		this.nextUsedTimeSlot = nextUsedTimeSlot;
		cancel();
		if(nextUsedTimeSlot != -1){
			long startTimeSlot = nextUsedTimeSlot * timeSlotLength + firstTimeSlot;
			long currentTime = System.currentTimeMillis();
			this.schedule(new NextProductStepTask(), startTimeSlot - currentTime);
		}
	}
	
	public long getNextUsedTimeSlot() {
		return nextUsedTimeSlot;
	}
	
	private class NextProductStepTask extends TimerTask {

		@Override
		public void run() {
			try {
				ObjectId productStepEntry = equipletAgent.getNextProductStep();
				String conversationId = equipletAgent.getConversationId(productStepEntry);
				BasicDBObject productStep = (BasicDBObject)equipletAgent.getEquipletBBClient().findDocumentById(productStepEntry);
				AID productAgent = new AID((String)productStep.get("productAgentId"), AID.ISGUID);

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