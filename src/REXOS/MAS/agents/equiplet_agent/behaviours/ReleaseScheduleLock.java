package agents.equiplet_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import agents.equiplet_agent.EquipletAgent;
import agents.equiplet_agent.ScheduleLock;
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
			// Create a response message
			ACLMessage response = message.createReply();

			// Get the ScheduleLock object for this equiplet
			ScheduleLock scheduleLock = equipletAgent.getScheduleLock();

			// Attempt to release the lock
			boolean lockReleased = scheduleLock
					.releaseLock(message.getSender());

			// If something went wrong while releasing the lock (if the product
			// agent attempting to release is not the owner for the lock) send a
			// failure message
			if (!lockReleased) {
				response.setPerformative(ACLMessage.FAILURE);
				equipletAgent.send(response);
			}
		}
	}

}
