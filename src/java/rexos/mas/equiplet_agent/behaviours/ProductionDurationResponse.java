package rexos.mas.equiplet_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
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
			Logger.log("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology());

			try {
				//gets the productstep
				ObjectId id = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStepMessage productStep = new ProductStepMessage((BasicDBObject) equipletBBClient.findDocumentById(id));

				ScheduleData schedule = productStep.getScheduleData();

				//sends a message to the productAgent with the production duration.
				ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
				responseMessage.addReceiver(productStep.getProductAgentId());
				responseMessage.setOntology("ProductionDuration");
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setContentObject(new Long(schedule.getDuration()));
				equipletAgent.send(responseMessage);
				
				ACLMessage scheduleStepMessage = new ACLMessage(ACLMessage.REQUEST);
				scheduleStepMessage.addReceiver(equipletAgent.getAID());
				scheduleStepMessage.setOntology("ScheduleStep");
				scheduleStepMessage.setConversationId(message.getConversationId());
				scheduleStepMessage.setContent(String.valueOf(5));
				equipletAgent.send(scheduleStepMessage);

				Logger.log("sending message: %s%n", responseMessage.getOntology());
				Logger.log("sending message: %s%n", scheduleStepMessage.getOntology());
			} catch (IOException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(e);
				equipletAgent.doDelete();
			}
		}
		equipletAgent.removeBehaviour(this);
	}
}
