package agents.equiplet_agent.behaviours;

import java.io.IOException;
import java.io.Serializable;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

public class CanPerformProductionStep extends ReceiveBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The messageTemplate this behaviour listens to.
	 *      This behaviour listens to the ontology: CanDoProductionStepResponse.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("CanPerformProductionStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;
	/**
	 * @var BlackboardClient equipletBBClient
	 * BlackboardClient for the product step blackboard.
	 */
	private BlackboardClient productStepBB;

	private ParentBehaviourCallback parentBehaviourCallback;
	
	private Object contentObject;
	private String conversationID;
	/**
	 * Instantiates a new can do production step response.
	 * 
	 * @param a
	 *      The agent for this behaviour
	 * @param productStepBB
	 * 		BlackboardClient for the product step blackboard.            
	 * @throws IOException Throws IOException when contentObject cannot be serialized
	 */
	public CanPerformProductionStep(EquipletAgent equipletAgent,
			BlackboardClient productStepBB, ParentBehaviourCallback parentBehaviourCallback,
			String conversationID, Object contentObject){
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent =  equipletAgent;
		this.productStepBB = productStepBB;
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.contentObject = contentObject;
		this.conversationID = conversationID;
	}
	
	@Override
	public void onStart(){
		ACLMessage queryIFMessage = new ACLMessage(ACLMessage.QUERY_IF);
		queryIFMessage.setConversationId(conversationID);
		queryIFMessage.addReceiver(equipletAgent.getServiceAgent());
		queryIFMessage.setOntology("CanPerformProductionStep");
		if (contentObject != null){
			try {
				queryIFMessage.setContentObject((Serializable)contentObject);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
		equipletAgent.send(queryIFMessage);
	
	}
	/**
	 * Function to handle the incoming messages for this behaviour.
	 * Handles the response to the CanPeformStep question and gives the result
	 * to the product agent.
	 * 
	 * @param message
	 *            - The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		
			//Logger.log(LogLevel.DEBUG, "%s received message from %s (%s)%n",
			//		myAgent.getLocalName(), message.getSender().getLocalName(),
			//		message.getOntology());

			ObjectId productStepEntryId = equipletAgent
					.getRelatedObjectId(message.getConversationId());
			
			if ( message.getPerformative() == ACLMessage.CONFIRM){
				//nothing special to do when the response is CONFIRM
			}
			
			// if the productstep can not be done by this equiplet remove it from the blackboard.
			else if (message.getPerformative() == ACLMessage.DISCONFIRM){
				try{
				productStepBB.removeDocuments(new BasicDBObject("_id",
						productStepEntryId));
				}catch (GeneralMongoException | InvalidDBNamespaceException e) {
					Logger.log(LogLevel.ERROR, e);
				}
			}
			
			//callback to the parent behaviour with the message
			parentBehaviourCallback.callback(message, null);
			equipletAgent.removeBehaviour(this);
				// gets the productstep from the blackboard to get the
				// productAgent out of it.
				
		
	}
}