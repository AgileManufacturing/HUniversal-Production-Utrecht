/**
 * 
 */
package serviceAgent.behaviour;

import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import serviceAgent.ServiceStepMessage;

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

	private BlackboardClient client;

	// private ServiceFactory factory;

	/**
	 * @param a
	 */
	public CanDoProductStep(Agent a, BlackboardClient client /*
															 * , ServiceFactory
															 * factory
															 */) {
		super(a, MessageTemplate.MatchOntology("EvaluateProductionStep"));
		this.client = client;
		// this.factory = factory;
	}

	/**
	 * @param a
	 * @param millis
	 */
	public CanDoProductStep(Agent a, int millis) {
		super(a, millis, MessageTemplate
				.MatchOntology("EvaluateProductionStep"));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage m) {
		// get the service for this productStepType from factory
		Service service = new DummyService();

		BasicDBObject productStep = null;
		try {
			productStep = (BasicDBObject) client.findDocumentById(
					(ObjectId) m.getContentObject());
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			e.printStackTrace();
		}
		long productStepType = productStep.getLong("type");
		BasicDBObject parameters = (BasicDBObject) productStep
				.get("parameters");

		ACLMessage reply = m.createReply();
		reply.setOntology("CanDoProductionStepResponse");
		if (service.canPerform(productStepType, parameters)) {
			m.setPerformative(ACLMessage.CONFIRM);
		} else {
			m.setPerformative(ACLMessage.DISCONFIRM);
		}
	}

	public static class DummyService implements Service {
		@Override
		public boolean canPerform(long productStepType, BasicDBObject parameters) {
			return true;
		}

		@Override
		public ServiceStepMessage[] getServiceSteps(long productStepType,
				BasicDBObject parameters) {
			return null;
		}

	}
}
