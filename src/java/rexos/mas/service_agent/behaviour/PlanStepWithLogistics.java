package rexos.mas.service_agent.behaviour;

import java.io.IOException;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.service_agent.ServiceAgent;

public class PlanStepWithLogistics extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private ServiceAgent agent;

	public PlanStepWithLogistics(Agent agent) {
		super(agent, MessageTemplate.MatchOntology("PlanStepWithLogistics"));
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
				ProductStepMessage productStep = new ProductStepMessage(
						(BasicDBObject) agent.getProductStepBBClient()
								.findDocumentById((ObjectId) message
										.getContentObject()));

				ACLMessage sendMsg = message.createReply();
				sendMsg.clearAllReceiver();
				sendMsg.addReceiver(agent.getLogisticsAID());
				sendMsg.setOntology("ArePartsAvailable");
				sendMsg.setPerformative(ACLMessage.QUERY_IF);
				sendMsg.setContentObject(productStep.getInputPartTypes());
				agent.send(sendMsg);

				agent.addBehaviour(new ArePartsAvailableResponse(agent, message
						.getConversationId(), (ObjectId) message
						.getContentObject(), productStep.getInputPartTypes()));
			} catch (InvalidDBNamespaceException | GeneralMongoException
					| UnreadableException | IOException e) {
				e.printStackTrace();
				agent.doDelete();
			}
		}
	}
}
