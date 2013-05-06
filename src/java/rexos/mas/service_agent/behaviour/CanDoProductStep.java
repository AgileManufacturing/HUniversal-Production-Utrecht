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
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ParameterGroup;
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
	public CanDoProductStep(Agent a) {
		super(a, MessageTemplate.MatchOntology("CanDoProductionStep"));
		agent = (ServiceAgent) a;
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
			ParameterGroup parameters = productStep.getParameters();

			System.out.format(
					"%s got message CanDoProductionStep for step type %s%n",
					agent.getLocalName(), stepType);

			 if (factory == null)
				 factory = new ServiceFactory(message.getSender().getLocalName());

			Service[] services = factory.getServicesForStep(stepType);
//			Service[] services = new Service[] { new PickAndPlaceService() };
			if (services.length > 0) {
				Service service = services[0];
				agent.MapConvIdWithService(message.getConversationId(), service);

				ACLMessage newMsg = message.createReply();
				newMsg.clearAllReceiver();
				newMsg.addReceiver(agent.getHardwareAgentAID());
				newMsg.setOntology("CheckForModules");
				try {
					//TODO change service.getModuleIds parameters to type ParameterGroup
					newMsg.setContentObject(service.getModuleIds(stepType,
							parameters.toBasicDBObject()));
				} catch (IOException e) {
					e.printStackTrace();
					agent.doDelete();
				}
				agent.send(newMsg);

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
			e.printStackTrace();
			agent.doDelete();
		}
	}
}
