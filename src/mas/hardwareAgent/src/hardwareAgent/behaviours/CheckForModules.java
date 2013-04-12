package hardwareAgent.behaviours;

/**
 * Author: Thierry Gerritse
 * Class: CheckForModules.java * 
 */

import java.sql.ResultSet;
import java.sql.SQLException;

import KnowledgeDBClient.KnowledgeDBClient;
import KnowledgeDBClient.Queries;
import KnowledgeDBClient.Row;
import behaviours.ReceiveBehaviour;
import hardwareAgent.HardwareAgent;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

public class CheckForModules extends ReceiveBehaviour {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate
			.MatchOntology("CheckForModules");
	private HardwareAgent hardwareAgent;

	/**
	 * 
	 * Instantiates a new check for module.
	 * s
	 */
	public CheckForModules(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
	}

	@Override
	public void handle(ACLMessage message) {
		Object contentObject = null;
		String contentString = message.getContent();
		try {

			try {
				contentObject = message.getContentObject();

				ACLMessage reply;
				reply = message.createReply();

				boolean modulesPresent = false;

				KnowledgeDBClient client = KnowledgeDBClient.getClient();

				ResultSet resultSet;

				resultSet = client.executeSelectQuery(Queries.MODULES);
				while (resultSet.next()) {
					System.out.println(new Row(resultSet));
					if (resultSet.equals(contentString)) {
						modulesPresent = true;
					}
				}
				// System.out.println();

				if (modulesPresent) {

					reply.setPerformative(ACLMessage.CONFIRM);

				} else {
					reply.setPerformative(ACLMessage.DISCONFIRM);
				}
				myAgent.send(reply);

				// TODO: check in knowledge database if the requested modules
				// are available
				// if available set performative (ACLMessage.Confirm) else set
				// performative (ACLMessage.Disconfirm)

			} catch (UnreadableException e) {
				// System.out.println("Exception Caught, No Content Object Given");
			} catch (SQLException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			System.out.format("%s received message from %s (%s:%s)%n", myAgent
					.getLocalName(), message.getSender().getLocalName(),
					message.getOntology(),
					contentObject == null ? contentString : contentObject);

		} catch (Exception e) {
			e.printStackTrace();
			// TODO: ERROR HANDLING
			myAgent.doDelete();
		}
	}
}
