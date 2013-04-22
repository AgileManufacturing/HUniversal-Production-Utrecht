/**
 * 
 */
package rexos.mas.service_agent.behaviour;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.service_agent.Service;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceFactory;

import com.mongodb.BasicDBObject;

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
		super(a, MessageTemplate.MatchOntology("CanDoProductionStep"));
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
			ProductStepMessage productStep = new ProductStepMessage(
					(BasicDBObject) client.findDocumentById(productStepId));
			long stepType = productStep.getType();
			BasicDBObject parameters = productStep.getParameters();

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
				newMsg.setContentObject(service.getModuleIds(stepType,
						parameters));
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
