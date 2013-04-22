package rexos.mas.equiplet_agent.behaviours;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.mas.equiplet_agent.ProductStepMessage;

import com.mongodb.BasicDBObject;

/**
 * The Class ProductionDurationResponse.
 */
public class ProductionDurationResponse extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ProductionDurationResponse");
	private EquipletAgent equipletAgent;
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new production duration response.
	 *
	 * @param a the a
	 */
	public ProductionDurationResponse(Agent a, BlackboardClient equipletBBClient) {
		super(a, -1, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.equipletBBClient = equipletBBClient;
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
		System.out.format("%s received message from %s (%s:%s)%n",
				myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(),
				contentObject == null ? contentString : contentObject);

		try {
			ObjectId id = equipletAgent.getRelatedObjectId(message.getConversationId());
			ProductStepMessage productStep = new ProductStepMessage((BasicDBObject)equipletBBClient.findDocumentById(id));

			ScheduleData schedule = productStep.getScheduleData();
			
			ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
			AID productAgent = productStep.getProductAgentId();
			responseMessage.addReceiver(productAgent);
			responseMessage.setOntology("ProductionDuration");
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.setContentObject(schedule.getDuration());
			myAgent.send(responseMessage);
			
			System.out.format("sending message: %s%n", responseMessage.getOntology());
			

		} catch (Exception e) {
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
