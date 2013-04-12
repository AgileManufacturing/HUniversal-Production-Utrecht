package serviceAgent.behaviour;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import serviceAgent.Service;
import serviceAgent.behaviour.CanDoProductStep.DummyService;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

import serviceAgent.ServiceStepMessage;

/**
 * @author Peter Bonnema
 *
 */
public class GetProductStepDuration extends ReceiveBehaviour {
	static final long serialVersionUID = 1L;

	private BlackboardClient client;
	//private ServiceFactory factory;

	/**
	 * @param a
	 */
	public GetProductStepDuration(Agent a, BlackboardClient client /*, ServiceFactory factory*/) {
		super(a);
		this.client = client;
//		this.factory = factory;
	}

	/* (non-Javadoc)
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
		
		ServiceStepMessage[] serviceSteps = service.getServiceSteps(productStepType, parameters);
		getAgent().addBehaviour(new GetServiceDuration(getAgent(), client, serviceSteps));
	}
}
