/**
 * 
 */
package serviceAgent.behaviour;

import java.io.IOException;

import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;

import serviceAgent.ServiceAgent;
import serviceAgent.ServiceFactory;
import serviceAgent.Service;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import behaviours.*;

/**
 * @author Peter Bonnema
 * 
 */
public class CanDoProductStep extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private ServiceAgent agent;
	private BlackboardClient client;
	private ServiceFactory factory;

	/**
	 * @param a
	 */
	public CanDoProductStep(Agent a, BlackboardClient client) {
		super(a, -1, MessageTemplate.MatchOntology("CanDoProductionStep"));
		agent = (ServiceAgent) a;
		this.client = client;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		try {
			ObjectId productStepId = (ObjectId) message.getContentObject();
			BasicDBObject productStep = (BasicDBObject) client
					.findDocumentById(productStepId);
			long stepType = 0;
			try {
				stepType = productStep.getLong("type");
			} catch (NullPointerException e) {
				e.printStackTrace();
				agent.doDelete();
			}
			BasicDBObject parameters = (BasicDBObject) productStep
					.get("parameters");

			System.out.format(
					"%s got message CanDoProductionStep for step type %s%n",
					agent.getLocalName(), stepType);

			if (factory == null)
				factory = new ServiceFactory(message.getSender().toString());
			Service service = factory.getServicesForStep(stepType)[0];

			ACLMessage newMsg = message.createReply();
			newMsg.clearAllReceiver();
			newMsg.addReceiver(agent.getHardwareAgentAID());
			newMsg.setOntology("CheckForModules");
			try {
				newMsg.setContentObject(service
						.getModuleIds(stepType, parameters));
			} catch (IOException e) {
				e.printStackTrace();
				agent.doDelete();
			}
			agent.send(newMsg);

			agent.addBehaviour(new CheckForModulesResponse(agent));
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			e.printStackTrace();
			agent.doDelete();
		}
	}
}
