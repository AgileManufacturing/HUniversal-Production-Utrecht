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

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
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
	private ServiceFactory factory;

	/**
	 * @param a
	 */
	public CanDoProductStep(Agent a, ServiceFactory factory) {
		super(a, MessageTemplate.MatchOntology("CanDoProductionStep"));
		agent = (ServiceAgent) a;
		this.factory = factory;
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
					(BasicDBObject) agent.getProductStepBBClient()
							.findDocumentById(productStepId));
			int stepType = productStep.getType();
			BasicDBObject parameters = productStep.getParameters();

			System.out.format(
					"%s got message CanDoProductionStep for step type %s%n",
					agent.getLocalName(), stepType);

			Service[] services = factory.getServicesForStep(stepType);
			if (services.length > 0) {
				Service service = services[0];
				agent.MapConvIdWithService(message.getConversationId(), service);

//				Queries.MODULEGROUPS_REQUIRED_PER_SERVICE;
				ACLMessage msg = new ACLMessage(ACLMessage.QUERY_IF);
				msg.setConversationId(message.getConversationId());
				msg.addReceiver(agent.getHardwareAgentAID());
				msg.setOntology("CheckForModules");
				try {
					msg.setContentObject(service.getModuleIds(stepType,
							parameters));
				} catch (IOException e) {
					Logger.log(e);
					agent.doDelete();
				}
				agent.send(msg);

				agent.addBehaviour(new CheckForModulesResponse(agent));
			} else {
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.DISCONFIRM);
				reply.setOntology("CanDoProductionStepResponse");
				getAgent().send(reply);
				System.out.format("%s sending step availability (%b)%n",
						getAgent().getLocalName(),
						reply.getPerformative() == ACLMessage.CONFIRM);
			}
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			Logger.log(e);
			agent.doDelete();
		}
	}
}
