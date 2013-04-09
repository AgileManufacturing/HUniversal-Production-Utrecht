package behaviours;

import org.bson.types.ObjectId;

import equipletAgent.EquipletAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;


public class GetProductionDuration extends ReceiveBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
    private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetProductionDuration");
    private EquipletAgent equipletAgent;
	
	public GetProductionDuration(Agent a) {
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
		
		
		
		try {
			ObjectId contentObjectId = equipletAgent.getCommunicationSlot(message.getConversationId());
			ACLMessage responseMessage = new ACLMessage(ACLMessage.REQUEST);
			responseMessage.addReceiver(equipletAgent.getServiceAgent());
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.setContentObject(contentObjectId);
			responseMessage.setOntology("getProductionStepDuration");

			/*try {

				responseMessage.setContentObject((Serializable) contentObject);

			} catch (IOException e) {

				System.out.println(e);
				e.printStackTrace();

			}*/

			myAgent.send(responseMessage);

		} catch (Exception e) {
			// TODO: ERROR HANDLING
			e.printStackTrace();
			myAgent.doDelete();
		}
	}
}
