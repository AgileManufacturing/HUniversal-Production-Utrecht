package serviceAgent.behaviour;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import serviceAgent.Service;
import serviceAgent.DummyService;
import serviceAgent.ServiceFactory;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import equipletAgent.ProductStepMessage;

import serviceAgent.ServiceStepMessage;

/**
 * @author Peter Bonnema
 * 
 */
public class GetProductStepDuration extends ReceiveBehaviour {
	static final long serialVersionUID = 1L;

	private BlackboardClient productionStepBlackBoard, serviceStepBlackBoard;

	/**
	 * @param a
	 */
	public GetProductStepDuration(Agent a,
			BlackboardClient productionStepBlackBoard,
			BlackboardClient serviceStepBlackBoard) {
		super(a);
		this.productionStepBlackBoard = productionStepBlackBoard;
		this.serviceStepBlackBoard = serviceStepBlackBoard;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		BasicDBObject productStep = null;

		try {
			productStep = (BasicDBObject) productionStepBlackBoard
					.findDocumentById((ObjectId) message.getContentObject());
		} catch (UnreadableException | InvalidDBNamespaceException
				| GeneralMongoException e) {
			e.printStackTrace();
		}
		long productStepType = productStep.getLong("type");

		Service[] services = null;
		Service service = null;
		ServiceFactory factory = new ServiceFactory(message.getSender()
				.getLocalName());
		if ((services = factory.getServicesForStep(productStepType)).length > 0)
			service = services[0];
		else
			throw new RuntimeException(
					"Service Agent - No available services for stepType "
							+ productStep);

		BasicDBObject parameters = (BasicDBObject) productStep
				.get("parameters");

		ServiceStepMessage[] serviceSteps = service.getServiceSteps(
				productStepType, parameters);
		getAgent().addBehaviour(
				new GetServiceDuration(getAgent(), productionStepBlackBoard,
						serviceStepBlackBoard, serviceSteps, msg
								.getConversationId()));
	}
}
