package equipletAgent.behaviours;

import org.bson.types.ObjectId;

import behaviours.ReceiveBehaviour;

import equipletAgent.EquipletAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * The Class GetProductionDuration.
 */
public class GetProductionDuration extends ReceiveBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
    private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetProductionDuration");
    private EquipletAgent equipletAgent;
	
	/**
	 * Instantiates a new gets the production duration.
	 *
	 * @param a the a
	 */
	public GetProductionDuration(Agent a) {
		super(a, -1, messageTemplate);
		equipletAgent = (EquipletAgent)a;
	}
	
	@Override
	public void handle(ACLMessage message){
		System.out.format("%s received message from %s%n",
				myAgent.getLocalName(), message.getSender().getLocalName(),
				message.getOntology());
		try {
			ObjectId contentObjectId = equipletAgent.getCommunicationSlot(message.getConversationId());
			ACLMessage responseMessage = new ACLMessage(ACLMessage.REQUEST);
			responseMessage.addReceiver(equipletAgent.getServiceAgent());
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.setContentObject(contentObjectId);
			responseMessage.setOntology("GetProductionStepDuration");
			myAgent.send(responseMessage);
		} catch (Exception e) {
			// TODO: ERROR HANDLING
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
