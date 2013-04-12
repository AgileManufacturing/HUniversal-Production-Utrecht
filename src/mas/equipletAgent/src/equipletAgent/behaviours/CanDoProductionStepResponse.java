package equipletAgent.behaviours;

import org.bson.types.ObjectId;
import com.mongodb.BasicDBObject;
import equipletAgent.StepStatusCode;
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
public class CanDoProductionStepResponse extends ReceiveBehaviour {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("CanDoProductionStepResponse");
	private EquipletAgent equipletAgent;

	/**
	 * Instantiates a new can do production step response.
	 * 
	 * @param a
	 *            the a
	 */
	public CanDoProductionStepResponse(Agent a) {
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
		System.out.format("%s received message from %s (%s:%s)%n", myAgent.getLocalName(),
				message.getSender().getLocalName(), message.getOntology(),
				contentObject == null ? contentString : contentObject);

		ObjectId productStepEntryId = equipletAgent.getCommunicationSlot(msg.getConversationId());
		try {
			BasicDBObject productStep = (BasicDBObject) equipletAgent.getEquipletBBclient().findDocumentById(productStepEntryId);
			StepStatusCode status = StepStatusCode.valueOf(productStep.getString("status"));
			AID productAgent = new AID(productStep.get("productAgentId").toString(), AID.ISLOCALNAME);
			if (status == StepStatusCode.EVALUATING) {
				ACLMessage responseMessage = new ACLMessage(ACLMessage.CONFIRM);
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setOntology("CanPerformStepResponse");
				responseMessage.addReceiver(productAgent);
				message.setContent("This is possible");
				myAgent.send(responseMessage);
			} else if (status == StepStatusCode.ABORTED) {
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
