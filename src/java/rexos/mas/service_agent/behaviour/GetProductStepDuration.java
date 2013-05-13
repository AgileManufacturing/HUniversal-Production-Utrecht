package rexos.mas.service_agent.behaviour;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.service_agent.Service;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceStepMessage;

import com.mongodb.BasicDBObject;

/**
 * @author Peter Bonnema
 * 
 */
public class GetProductStepDuration extends ReceiveBehaviour {
	static final long serialVersionUID = 1L;

	private ServiceAgent agent;

	/**
	 * @param a
	 */
	public GetProductStepDuration(Agent a) {
		super(a, MessageTemplate.MatchOntology("GetProductionStepDuration"));
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
			int productStepType = productStep.getType();

			System.out.format(
					"%s got message GetProductStepDuration for step type %s%n",
					agent.getLocalName(), productStepType);

			Service service = agent.GetServiceForConvId(message.getConversationId());
			BasicDBObject parameters = productStep.getParameters();
			ServiceStepMessage[] serviceSteps = service.getServiceSteps(
					productStepType, parameters);
			for (ServiceStepMessage serviceStep : serviceSteps) {
				serviceStep.setProductStepId(productStepId);
			}

			agent.addBehaviour(new GetServiceDuration(agent, serviceSteps,
					message.getConversationId()));
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			e.printStackTrace();
			agent.doDelete();
		}
	}
}
