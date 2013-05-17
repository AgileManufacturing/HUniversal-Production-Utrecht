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
import rexos.mas.service_agent.ServiceAgent;

import com.mongodb.BasicDBObject;

public class ScheduleStep extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private ServiceAgent agent;

	public ScheduleStep(Agent agent) {
		super(agent, MessageTemplate.MatchOntology("ScheduleStep"));
		this.agent = (ServiceAgent) agent;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				Logger.log("%s scheduling step with Logistics%n", agent.getLocalName());
				
				ProductStepMessage productStep = new ProductStepMessage(
						(BasicDBObject) agent.getProductStepBBClient()
								.findDocumentById((ObjectId) message
										.getContentObject()));
				
				ACLMessage sendMsg = new ACLMessage(ACLMessage.QUERY_IF);
				sendMsg.setConversationId(message.getConversationId());
				sendMsg.addReceiver(agent.getLogisticsAID());
				sendMsg.setOntology("ArePartsAvailable");
				sendMsg.setContentObject(productStep);
				agent.send(sendMsg);

				agent.addBehaviour(new ArePartsAvailableResponse(agent, message
						.getConversationId(), productStep));
			} catch (InvalidDBNamespaceException | GeneralMongoException
					| UnreadableException | IOException e) {
				Logger.log(e);
				agent.doDelete();
			}
		}
	}
}
