package equipletAgent.behaviours;

import newDataClasses.ScheduleData;
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
 * The Class ProductionDurationResponse.
 */
public class ProductionDurationResponse extends ReceiveBehaviour {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ProductionDurationResponse");
	private EquipletAgent equipletAgent;

	/**
	 * Instantiates a new production duration response.
	 *
	 * @param a the a
	 */
	public ProductionDurationResponse(Agent a) {
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

		try {
			ObjectId id = equipletAgent.getCommunicationSlot(message.getConversationId());
			DBObject productStep = equipletAgent.getEquipletBBclient().findDocumentById(id);

			ScheduleData schedule = new ScheduleData(); 
			schedule.fromBasicDBObject(((BasicDBObject)productStep.get("scheduleData")));
			System.out.println(schedule.getDuration() + "");
			ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
			responseMessage.addReceiver(new AID(productStep.get("productAgentId").toString(), AID.ISLOCALNAME));
			responseMessage.setOntology("ProductionDuration");
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.setContentObject(schedule.getDuration());

			System.out.format("sending message: %s%n", responseMessage);
			myAgent.send(responseMessage);

		} catch (Exception e) {
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
