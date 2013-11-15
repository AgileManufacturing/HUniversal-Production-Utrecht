package agents.equiplet_agent.behaviours;

import java.util.UUID;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

public class ReleaseScheduleLock extends ReceiveBehaviour {

	/**
	 * @var static final long serialVersionUID The serial version UID for this
	 *      class
	 */
	private static final long serialVersionUID = 7310593349133854636L;

	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate
			.MatchOntology("ReleaseScheduleLock");

	private EquipletAgent equipletAgent;

	public ReleaseScheduleLock(EquipletAgent equipletAgent) {
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent = equipletAgent;
	}

	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try{
				// Create a response message
				ACLMessage response = message.createReply();
				message.setPerformative(ACLMessage.AGREE);
				// Get the ScheduleLock for the equiplet and release it with the given key. 
				UUID key = (UUID) message.getContentObject();
				boolean lockReleased = equipletAgent.getSchedule().releaseScheduleLock(key);
				
				if (!lockReleased) {
					response.setPerformative(ACLMessage.REFUSE);
					equipletAgent.send(response);
				}
			}catch(UnreadableException e){
				ACLMessage response = message.createReply();
				response.setPerformative(ACLMessage.FAILURE);
				equipletAgent.send(response);
			}
		}
	}

}
