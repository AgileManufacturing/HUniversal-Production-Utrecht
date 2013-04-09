package behaviours;

import hardwareAgent.HardwareAgent;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class EvaluateDuration extends ReceiveBehaviour {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("EvaluateDuration");
	private HardwareAgent hardwareAgent;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a
	 *            the a
	 */
	public EvaluateDuration(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
	}

	@Override
	public void handle(ACLMessage message) {
		Object contentObject = null;
		String contentString = message.getContent();

		try {
			contentObject = message.getContentObject();
		} catch (UnreadableException e) {
			// System.out.println("Exception Caught, No Content Object Given");
		}
		System.out.format("%s received message from %s (%s:%s)%n", myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(), contentObject == null ? contentString : contentObject);

		try {
			ObjectId objectId = null;
			DBObject productionStep = null;
			try {
				objectId = (ObjectId) message.getContentObject();
				BasicDBObject query = new BasicDBObject();
				query.put("_id", objectId);
				productionStep = hardwareAgent.getServiceStepsBBClient().findDocuments(query).get(0);
			} catch (UnreadableException | InvalidDBNamespaceException e) {
				e.printStackTrace();
				myAgent.doDelete();
			}
			if (productionStep != null) {
				int type = ((Integer) productionStep.get("type"));
				Object parameters = productionStep.get("parameters");
				switch (type) {
				case 1:// pickup
					PickUpTranslation(parameters);
					break;
				case 2:// drop
					DropTranslation(parameters);
					break;
				default:
					break;
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private boolean PickUpTranslation(Object parameters) {
		// double x = parameters.getX();
		// double y = parameters.getY();
		// double z = parameters.getZ();
		// long relative = parameters.getRelative();

		// Move delta robot to *,*,8 relative to crate A(relative)
		// Move delta robot to x,y,* relative to crate A(relative)
		// Move delta robot to *,*,z relative to crate A(relative)
		// Activate gripper
		// Move delta robot to *,*,8 relative to crate A(relative)
		return true;
	}

	private boolean DropTranslation(Object parameters) {
		// double x = parameters.getX();
		// double y = parameters.getY();
		// double z = parameters.getZ();
		// long relative = parameters.getRelative();

		// Move delta robot to *,*,8 relative to crate A
		// Move delta robot to x,y,* relative to crate A
		// Move delta robot to *,*,z relative to crate A
		// Deactivate gripper
		// Move delta robot to *,*,8 relative to crate A
		return true;
	}
}
