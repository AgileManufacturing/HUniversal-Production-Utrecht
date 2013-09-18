package rexos.mas.equiplet_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.LogLevel;
import rexos.mas.data.ProductStep;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.utillities.log.Logger;

import com.mongodb.BasicDBObject;

/**
 * A receive once behaviour for receiving messages with the ontology: "ProductionDurationResponse".
 * Receives the message from its service agent and sends the productduration to the product agent.
 * The message send to the product agent has the ontology: "ProductionDuration".
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
		super(a, 10000, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.equipletBBClient = equipletBBClient;
	}

	/**
	 * Function for handling the incoming of messages.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			Logger.logAclMessage(message, 'r');
			//Logger.log("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(),
					//message.getOntology());

			try {
				// gets the productstep
				ObjectId id = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStep productStep = new ProductStep((BasicDBObject) equipletBBClient.findDocumentById(id));

				ScheduleData schedule = productStep.getScheduleData();
//				schedule.setDuration(schedule.getDuration() + (6000/equipletAgent.getTimer().getTimeSlotLength()));
//				equipletBBClient.updateDocuments(new BasicDBObject("_id", id),
//												new BasicDBObject("$set", new BasicDBObject("scheduleData.duration", schedule.getDuration())));
				// sends a message to the productAgent with the production
				// duration.
				ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
				responseMessage.addReceiver(productStep.getProductAgentId());
				responseMessage.setOntology("ProductionDuration");
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setContentObject(new Long(schedule.getDuration()));
				equipletAgent.send(responseMessage);
				Logger.logAclMessage(responseMessage, 's');
				Logger.log(LogLevel.DEBUG, "sending message: %s%n", responseMessage.getOntology());
			} catch(IOException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
				equipletAgent.doDelete();
			}
		}else{
			Logger.log(LogLevel.ERROR, "equiplet agent - duration response timed out");
		}
	}
}
