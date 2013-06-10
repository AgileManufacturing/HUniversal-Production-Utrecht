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
import rexos.mas.equiplet_agent.ProductStep;

import com.mongodb.BasicDBObject;

/**
 * The Class ProductionDurationResponse.
 */
public class ProductionDurationResponse extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 **/
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The MessageTemplate indicating the messages this behaviour wishes to
	 *      receive.
	 **/
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ProductionDurationResponse");

	/**
	 * @var EquipletAgent equipletAgent
	 *      A reference to the equipletAgent this behaviour belongs to.
	 **/
	private EquipletAgent equipletAgent;

	/**
	 * @var BlackboardClient equipletBBClient
	 *      The blackboardclient for this equiplet's blackboard.
	 **/
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new production duration response.
	 * 
	 * @param a
	 *            the agent
	 * @param equipletBBClient
	 *            the equiplet blackboard.
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
				// gets the productstep
				ObjectId id = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStep productStep = new ProductStep((BasicDBObject) equipletBBClient.findDocumentById(id));

				ScheduleData schedule = productStep.getScheduleData();

				// sends a message to the productAgent with the production
				// duration.
				ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
				responseMessage.addReceiver(productStep.getProductAgentId());
				responseMessage.setOntology("ProductionDuration");
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setContentObject(new Long(schedule.getDuration()));
				equipletAgent.send(responseMessage);
				Logger.log("sending message: %s%n", responseMessage.getOntology());

				//TODO: remove below after testing
				ACLMessage scheduleStepMessage = new ACLMessage(ACLMessage.REQUEST);
				scheduleStepMessage.addReceiver(equipletAgent.getAID());
				scheduleStepMessage.setOntology("ScheduleStep");
				scheduleStepMessage.setConversationId(message.getConversationId());
				scheduleStepMessage.setContent(String.valueOf((System.currentTimeMillis() - equipletAgent.getTimer().getFirstTimeSlot())/2000 + 3));
				equipletAgent.send(scheduleStepMessage);
				Logger.log("sending message: %s%n", scheduleStepMessage.getOntology());
				//TODO: remove above after testing
			} catch (IOException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(e);
				equipletAgent.doDelete();
			}
		}
		equipletAgent.removeBehaviour(this);
	}
}
