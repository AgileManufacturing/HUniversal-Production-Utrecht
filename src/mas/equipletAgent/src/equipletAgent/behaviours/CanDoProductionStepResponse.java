package equipletAgent.behaviours;

import java.lang.reflect.Type;

import org.bson.types.ObjectId;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import equipletAgent.ProductStepStatusCode;
import equipletAgent.EquipletAgent;
import behaviours.ReceiveBehaviour;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * The Class CanDoProductionStepResponse.
 */
public class CanDoProductionStepResponse extends ReceiveBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
    private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("CanDoProductionStepResponse");
    private EquipletAgent equipletAgent;
	
	/**
	 * Instantiates a new can do production step response.
	 *
	 * @param a the a
	 */
	public CanDoProductionStepResponse(Agent a) {
		super(a, -1, messageTemplate);
		equipletAgent = (EquipletAgent)a;
	}
	
	@Override
	public void handle(ACLMessage message){
		Object contentObject = null;
		String contentString = message.getContent();

		try {
			contentObject = message.getContentObject();
		} catch (UnreadableException e) {
			//System.out.println("Exception Caught, No Content Object Given");
		}
		System.out.format("%s received message from %s (%s:%s)%n",
				myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(), contentObject == null ? contentString : contentObject);
		
		ObjectId productStepEntryId = null;
		DBObject productStep = null;
		Gson gson = new GsonBuilder()
			.registerTypeAdapter(jade.util.leap.List.class, new InstanceCreator<jade.util.leap.List>() {
				@Override
				public jade.util.leap.List createInstance(Type type) {
					return new jade.util.leap.ArrayList();
				}
			})
			.create();
		try {
			productStepEntryId = (ObjectId) contentObject;
		} catch (ClassCastException e) {
			// TODO: ERROR HANDLING
			e.printStackTrace();
			myAgent.doDelete();
		}
		// Makes a database connection and gets the right
		// product step out of it.
		try {
			BasicDBObject query = new BasicDBObject();
			query.put("_id", productStepEntryId);
			productStep = equipletAgent.getEquipletBBclient().findDocuments(query).get(0);
			int status = (Integer) productStep.get("status");
			AID productAgent = gson.fromJson(productStep.get("productAgentId").toString(), AID.class);
			if (status == ProductStepStatusCode.EVALUATING.getStatus()) {
				ACLMessage responseMessage = new ACLMessage(ACLMessage.CONFIRM);
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setOntology("CanPerformStepResponse");
				responseMessage.addReceiver(productAgent);
				message.setContent("This is possible");
				myAgent.send(responseMessage);
			} else if (status == ProductStepStatusCode.ABORTED.getStatus()) {
				ACLMessage responseMessage = new ACLMessage(ACLMessage.DISCONFIRM);
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setOntology("CanPerformStepResponse");
				responseMessage.addReceiver(productAgent);
				responseMessage.setContent("This is impossible");
				myAgent.send(responseMessage);
			}
		} catch (Exception e) {
			// TODO: ERROR HANDLING
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
