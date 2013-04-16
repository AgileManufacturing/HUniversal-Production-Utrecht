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

import serviceAgent.ServiceFactory;
import serviceAgent.Service;
import jade.core.Agent;
import jade.core.behaviours.SenderBehaviour;
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

	private BlackboardClient client;
	private ServiceFactory factory;

	/**
	 * @param a
	 */
	public CanDoProductStep(Agent a, BlackboardClient client) {
		super(a, -1, MessageTemplate.MatchOntology("CanDoProductionStep"));
		this.client = client;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		ObjectId productStepId = null;
		BasicDBObject productStep = null;
		try {
			productStepId = (ObjectId) message.getContentObject();
			productStep = (BasicDBObject) client
					.findDocumentById(productStepId);
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			e.printStackTrace();
			getAgent().doDelete();
		}
		long stepType = productStep.getLong("type");
		BasicDBObject parameters = (BasicDBObject) productStep
				.get("parameters");
		
		System.out.format(
				"%s got message CanDoProductionStep for step type %s%n",
				getAgent().getLocalName(), productStep.get("type"));

		if (factory == null)
			factory = new ServiceFactory(message.getSender().toString());
		Service service = factory.getServicesForStep(stepType)[0];

		ACLMessage reply = message.createReply();
		reply.setOntology("CanDoProductionStepResponse");
		try {
			reply.setContentObject(productStepId);
		} catch (IOException e) {
			e.printStackTrace();
			getAgent().doDelete();
		}
		if (service.canPerform(stepType, parameters)) {
			message.setPerformative(ACLMessage.CONFIRM);
			System.out.format("%s sending can do step%n", getAgent()
					.getLocalName());
		} else {
			message.setPerformative(ACLMessage.DISCONFIRM);
			System.out.format("%s sending cannot do step%n", getAgent()
					.getLocalName());
		}
		getAgent().send(reply);
	}
}
