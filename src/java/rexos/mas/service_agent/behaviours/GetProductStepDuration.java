package rexos.mas.service_agent.behaviours;

import java.io.IOException;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
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

	/* (non-Javadoc)
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage) */
	@Override
	public void handle(ACLMessage message) {
		try {
			ObjectId productStepId = (ObjectId) message.getContentObject();
			ProductStepMessage productStep =
					new ProductStepMessage((BasicDBObject) agent.getProductStepBBClient().findDocumentById(
							productStepId));
			int productStepType = productStep.getType();

			Logger.log("%s got message GetProductStepDuration for step type %s%n", agent.getLocalName(),
					productStepType);

			Service service = agent.GetServiceForConvId(message.getConversationId());
			BasicDBObject parameters = productStep.getParameters();
			ServiceStepMessage[] serviceSteps = service.getServiceSteps(productStepType, parameters);
			for(ServiceStepMessage serviceStep : serviceSteps) {
				serviceStep.setProductStepId(productStepId);
			}
			
			Logger.log("%s asking %s for duration of %d steps%n", agent.getLocalName(), agent.getHardwareAgentAID()
					.getLocalName(), serviceSteps.length);

			ObjectId serviceStepId = null;
			ACLMessage askMessage = new ACLMessage(ACLMessage.QUERY_IF);
			askMessage.addReceiver(agent.getHardwareAgentAID());
			askMessage.setOntology("GetServiceStepDuration");
			askMessage.setConversationId(message.getConversationId());
			ServiceStepMessage serviceStep;
			BlackboardClient serviceStepBB = agent.getServiceStepBBClient();
			for(int i = serviceSteps.length - 1; i >= 0; i--) {
				serviceStep = serviceSteps[i];
				serviceStep.setNextStep(serviceStepId);
				serviceStepId = serviceStepBB.insertDocument(serviceStep.toBasicDBObject());
			}
			askMessage.setContentObject(serviceStepId);
			agent.send(askMessage);

			agent.addBehaviour(new GetServiceStepsDuration(agent, message.getConversationId()));
		} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException | IOException e) {
			Logger.log(e);
			agent.doDelete();
		}
	}
}
