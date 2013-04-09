package behaviours;

import java.lang.reflect.Type;

import org.bson.types.ObjectId;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import equipletAgent.EquipletAgent;
import equipletAgent.ScheduleData;
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
			Gson gson = new GsonBuilder().registerTypeAdapter(jade.util.leap.List.class, new InstanceCreator<jade.util.leap.List>() {
				@Override
				public jade.util.leap.List createInstance(Type type) {
					return new jade.util.leap.ArrayList();
				}
			}).create();
			ObjectId contentObjectId = equipletAgent.getCommunicationSlot(message.getConversationId());
			BasicDBObject query = new BasicDBObject();
			query.put("_id", contentObjectId);
			DBObject productStep = equipletAgent.getEquipletBBclient().findDocuments(query).get(0);

			ScheduleData Schedule = gson.fromJson(productStep.get("scheduleData").toString(), ScheduleData.class);

			ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
			responseMessage.addReceiver(gson.fromJson(productStep.get("productAgentId").toString(), AID.class));
			responseMessage.setOntology("ProductionDuration");
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.setContentObject(Schedule.getDuration());

			System.out.format("sending message: %s%n", responseMessage);
			myAgent.send(responseMessage);

		} catch (Exception e) {
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
