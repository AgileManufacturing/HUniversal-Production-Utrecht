package rexos.mas.equiplet_agent.behaviours;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.mas.equiplet_agent.ProductStepMessage;

import com.mongodb.BasicDBObject;

/**
 * The Class ProductionDurationResponse.
 */
public class ProductionDurationResponse extends ReceiveOnceBehaviour {
	private static final long serialVersionUID = 1L;

	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ProductionDurationResponse");
	private EquipletAgent equipletAgent;
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new production duration response.
	 * 
	 * @param a
	 *            the a
	 */
	public ProductionDurationResponse(Agent a, BlackboardClient equipletBBClient) {
		super(a, 5000, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.equipletBBClient = equipletBBClient;
	}

	/**
	 * Function for handling the incoming of messages.
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			System.out.format("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology());

			try {
				//gets the productstep
				ObjectId id = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStepMessage productStep = new ProductStepMessage((BasicDBObject) equipletBBClient.findDocumentById(id));

				ScheduleData schedule = productStep.getScheduleData();

				//sends a message to the productAgent with the production duration.
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
		myAgent.removeBehaviour(this);
	}
}
