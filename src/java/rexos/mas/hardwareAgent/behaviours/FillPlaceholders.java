package rexos.mas.hardwareAgent.behaviours;

import rexos.mas.hardwareAgent.HardwareAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import rexos.libraries.blackboard_client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import rexos.mas.behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class FillPlaceholders extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("FillPlaceholders");
	private HardwareAgent hardwareAgent;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a
	 *            the a
	 */
	public FillPlaceholders(Agent a) {
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
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
