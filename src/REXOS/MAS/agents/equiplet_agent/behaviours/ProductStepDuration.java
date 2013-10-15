package agents.equiplet_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;
import java.io.Serializable;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.data_classes.ScheduleData;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveOnceBehaviour;

import com.mongodb.BasicDBObject;

/**
 * A receive once behaviour for receiving messages with the ontology: "ProductionDurationResponse".
 * Receives the message from its service agent and sends the productduration to the product agent.
 * The message send to the product agent has the ontology: "ProductionDuration".
 */
public class ProductStepDuration extends ReceiveOnceBehaviour {
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
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ProductStepDuration");

	/**
	 * @var EquipletAgent equipletAgent
	 *      A reference to the equipletAgent this behaviour belongs to.
	 **/
	private EquipletAgent equipletAgent;

	/**
	 * @var BlackboardClient equipletBBClient
	 *      The blackboardclient for this equiplet's blackboard.
	 **/
	private BlackboardClient productStepBB;

	private ParentBehaviourCallback parentBehaviourCallback;

	private Object contentObject;

	private String conversationID;

	/**
	 * Instantiates a new production duration response.
	 * 
	 * @param a
	 *            the agent
	 * @param equipletBBClient
	 *            the productSteps blackboard for the equiplet.
	 */
	public ProductStepDuration(EquipletAgent equipletAgent, BlackboardClient productStepBB, ParentBehaviourCallback parentBehaviourCallback,
			String conversationID, Object contentObject) {
		super(equipletAgent, 10000, messageTemplate);
		this.equipletAgent =  equipletAgent;
		this.productStepBB = productStepBB;
		
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.contentObject = contentObject;
		this.conversationID = conversationID;
	}
	
	@Override
	public void onStart(){
		ACLMessage responseMessage = new ACLMessage(ACLMessage.QUERY_REF);
		responseMessage.addReceiver(equipletAgent.getServiceAgent());
		responseMessage.setConversationId(conversationID);
		if (contentObject != null){
			try {
				responseMessage.setContentObject((Serializable)contentObject);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
		responseMessage.setOntology("ProductStepDuration");
		equipletAgent.send(responseMessage);
	}

	/**
	 * Function for handling the incoming of messages.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			//Logger.log("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(),
					//message.getOntology());

			try {
				// gets the productstep
				ObjectId productStepId = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStep productStep = new ProductStep((BasicDBObject) productStepBB.findDocumentById(productStepId));

				ScheduleData schedule = productStep.getScheduleData();
//				schedule.setDuration(schedule.getDuration() + (6000/equipletAgent.getTimer().getTimeSlotLength()));
//				equipletBBClient.updateDocuments(new BasicDBObject("_id", id),
//												new BasicDBObject("$set", new BasicDBObject("scheduleData.duration", schedule.getDuration())));
				
				
				BehaviourCallbackItem scheduleArguments = new BehaviourCallbackItem();
				scheduleArguments.addArgument("schedule", schedule);
				scheduleArguments.addArgument("productStep", productStep);
				
				parentBehaviourCallback.callback(message, scheduleArguments);
				equipletAgent.removeBehaviour(this);
				
		//		Logger.log(LogLevel.DEBUG, "sending message: %s%n", responseMessage.getOntology());
			} catch(InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
				equipletAgent.doDelete();
			}
		}else{
			Logger.log(LogLevel.ERROR, "equiplet agent - duration response timed out");
		}
	}
}
