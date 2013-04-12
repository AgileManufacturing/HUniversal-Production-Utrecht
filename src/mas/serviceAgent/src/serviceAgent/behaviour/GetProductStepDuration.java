package serviceAgent.behaviour;

import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SenderBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import java.io.IOException;
import java.util.Hashtable;

import newDataClasses.ScheduleData;
import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;
import nl.hu.client.InvalidJSONException;

import org.bson.types.ObjectId;

import serviceAgent.Service;
import serviceAgent.ServiceAgent;
import serviceAgent.behaviour.CanDoProductStep.DummyService;

import behaviours.ReceiveBehaviour;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
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
		getAgent().addBehaviour(new GetServiceDurationBehaviour(getAgent(), client, serviceSteps));
	}
}
