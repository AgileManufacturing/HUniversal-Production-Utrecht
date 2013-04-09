package hardwareAgent;

import behaviours.EvaluateDuration;

import equipletAgent.DbData;
import jade.core.Agent;
import nl.hu.client.BasicOperationSubscription;
import nl.hu.client.BlackboardClient;
import nl.hu.client.BlackboardSubscriber;
import nl.hu.client.MongoOperation;
import nl.hu.client.OplogEntry;

public class HardwareAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;

	private BlackboardClient serviceStepBBClient, equipletStepBBClient;
	private DbData dbData;

	public void setup() {
		System.out.println("I spawned as a service agent.");

		// TODO fill in host, database and collection
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
		}

		try {
			serviceStepBBClient = new BlackboardClient(dbData.ip);
			serviceStepBBClient.setDatabase(dbData.name);
			serviceStepBBClient.setCollection("ServiceStepsBlackboard");
			serviceStepBBClient.subscribe(new BasicOperationSubscription(MongoOperation.INSERT, this));
			
			equipletStepBBClient = new BlackboardClient(dbData.ip);
			equipletStepBBClient.setDatabase(dbData.name);
			equipletStepBBClient.setCollection("EquipletStepsBlackboard");
		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}
		
		EvaluateDuration evaluateDurationBehaviour = new EvaluateDuration(this);
		addBehaviour(evaluateDurationBehaviour);
	}

	public void takeDown() {
		// TODO implement graceful death
	}

	public BlackboardClient getServiceStepsBBClient(){
		return serviceStepBBClient;
	}
	
	
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		switch (entry.getNamespace().split(".")[1]) {
		case "ServiceStepsBlackboard":

			switch (operation) {
			case INSERT:
				break;
			case UPDATE:
				break;
			case DELETE:
				break;
			default:
				break;
			}
			break;
		case "EquipletStepsBlackboard":
			break;
		}

	}

}
