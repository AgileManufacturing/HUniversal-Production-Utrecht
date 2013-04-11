package equipletAgent.behaviours;


import org.bson.types.ObjectId;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import equipletAgent.EquipletAgent;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * The Class ScheduleStep.
 */
public class ScheduleStep extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ScheduleStep");
	private EquipletAgent equipletAgent;

	/**
	 * Instantiates a new schedule step.
	 *
	 * @param a the a
	 */
	public ScheduleStep(Agent a) {
		super(a, -1, messageTemplate);
		equipletAgent = (EquipletAgent) a;
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

		try{
			
			long timeslot = Long.parseLong(contentString);
			ObjectId contentObjectId = equipletAgent.getCommunicationSlot(message.getConversationId());
			BasicDBObject query = new BasicDBObject("_id", contentObjectId);
			DBObject productStep = equipletAgent.getEquipletBBclient().findDocuments(query).get(0);
			System.out.format("%d%n", timeslot);
			ACLMessage timeslotMessage = new ACLMessage(ACLMessage.REQUEST);
			timeslotMessage.addReceiver(equipletAgent.getServiceAgent());
			timeslotMessage.setOntology("PlanStepWithLogistics");
			timeslotMessage.setContent(String.valueOf(timeslot));
			timeslotMessage.setConversationId(message.getConversationId());
			myAgent.send(timeslotMessage);
			ACLMessage confirmScheduleStep = new ACLMessage(ACLMessage.CONFIRM);
			confirmScheduleStep.setConversationId(message.getConversationId());
			confirmScheduleStep.addReceiver((AID) productStep.get("productAgentId"));
		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
}
