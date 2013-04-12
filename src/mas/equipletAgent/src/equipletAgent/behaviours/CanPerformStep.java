package equipletAgent.behaviours;

import java.io.IOException;

import newDataClasses.ProductionStep;

import org.bson.types.ObjectId;

import behaviours.ReceiveBehaviour;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.mongodb.DBObject;

import equipletAgent.ProductStepMessage;
import equipletAgent.StepStatusCode;
import equipletAgent.EquipletAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * The Class CanPerformStep.
 */
public class CanPerformStep extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	
    private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("CanPerformStep");
    private EquipletAgent equipletAgent;
	
	/**
	 * Instantiates a new can perform step.
	 *
	 * @param a the a
	 */
	public CanPerformStep(Agent a) {
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
		
		
		ProductionStep proStepC = (ProductionStep) contentObject;
		ObjectId productStepEntryId = null;
		Gson gson = new GsonBuilder().serializeNulls().create();

		try {
			// TODO: get inputParts
			// TODO: get ouputPart
			ProductStepMessage entry = new ProductStepMessage(message.getSender(), proStepC.getCapability(),
					proStepC.getParameterList(), null, null,
					StepStatusCode.EVALUATING.getStatus(), null, null);
			productStepEntryId = equipletAgent.getEquipletBBclient().insertDocument(gson.toJson(entry));
			equipletAgent.addCommunicationSlot(message.getConversationId(), productStepEntryId);
			// Asks the serviceAgent if it can do this product step.
			ACLMessage responseMessage = new ACLMessage(ACLMessage.REQUEST);
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.addReceiver(equipletAgent.getServiceAgent());
			responseMessage.setOntology("CanDoProductionStep");
			try {
				responseMessage.setContentObject(productStepEntryId);
			} catch (IOException e) {
				e.printStackTrace();
				// TODO: ERROR HANDLING
				myAgent.doDelete();
			}
			myAgent.send(responseMessage);
		} catch (Exception e) {
			e.printStackTrace();
			// TODO: ERROR HANDLING
			myAgent.doDelete();
		}
	}
}
