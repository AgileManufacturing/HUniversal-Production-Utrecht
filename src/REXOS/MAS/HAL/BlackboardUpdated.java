package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import HAL.listeners.BlackboardListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.BlackboardSubscriber;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.MongoOperation;
import libraries.blackboard_client.data_classes.OplogEntry;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription.MongoUpdateLogOperation;
import com.mongodb.DBObject;

public class BlackboardUpdated implements BlackboardSubscriber {
	private ArrayList<BlackboardListener> updateSubscribers;
	private BlackboardClient equipletStepBBClient;
	private BlackboardClient stateBlackboardBBClient;
	private FieldUpdateSubscription stateSubscription;
	private FieldUpdateSubscription modeSubscription;
	private BlackboardClient modeBlackboardBBClient;
	
	String equipletName = null;
	String state = null;
	String mode = null;
	String status;

	private FieldUpdateSubscription statusSubscription;

	public BlackboardUpdated(){
		updateSubscribers = new ArrayList<BlackboardListener>();
			
		try {
			
//			serviceStepBBClient = new BlackboardClient(dbData.getIp());
//			serviceStepBBClient.setDatabase(dbData.getName());
//			serviceStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ServiceStepsBlackBoardName", equipletAgentAID.getLocalName()));
//
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			stateSubscription = new FieldUpdateSubscription("state", this);
			stateSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			modeSubscription = new FieldUpdateSubscription("mode", this);
			modeSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			equipletStepBBClient = new BlackboardClient("145.89.191.131");
			equipletStepBBClient.setDatabase("EQ1");
			equipletStepBBClient.setCollection("EquipletStepsBlackBoard");
			equipletStepBBClient.subscribe(statusSubscription);
			
			stateBlackboardBBClient = new BlackboardClient("145.89.191.131");
			stateBlackboardBBClient.setDatabase("StateBlackboard");
			stateBlackboardBBClient.setCollection("equipletState");
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			modeBlackboardBBClient = new BlackboardClient("145.89.191.131");
			modeBlackboardBBClient.setDatabase("StateBlackboard");
			modeBlackboardBBClient.setCollection("equipletState");
			modeBlackboardBBClient.subscribe(modeSubscription);

			
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void setOnBlackBoardUpdatedListener(BlackboardListener blackboardListener){
		updateSubscribers.add(blackboardListener);
		
	}

	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		// TODO Auto-generated method stub
		
		DBObject dbObject;
		try{
			switch (entry.getNamespace().split("\\.")[1]) {
			
			case "equipletState":
				dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					equipletName = dbObject.get("equipletName").toString();
					if(dbObject.containsField("state")){
						state = dbObject.get("state").toString();
					}
					if(dbObject.containsField("mode")){
						mode = dbObject.get("mode").toString();
					}	
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.OnEquipleStateChanged(equipletName,state);
						changedListener.OnEquipleModeChanged(equipletName, mode);
					}
				}
					
				break;
				
//			case "ProductStepsBlackboard":
//				dbObject = productStepBBClient.findDocumentById(entry.getTargetObjectId());
//				if(dbObject != null) {
//					status = dbObject.get("status").toString();
//					Logger.log(LogLevel.DEBUG, "Ari ProductStep status set to: %s%n", status);
//					for(BlackboardListener changedListener: updateSubscribers){
//						changedListener.onProcessStateChanged(status);
//					}
//				}
//				
//				break;
//				
//			case "ServiceStepsBlackboard":
//				dbObject = serviceStepBBClient.findDocumentById(entry.getTargetObjectId());
//				if(dbObject != null) {
//					status = dbObject.get("status").toString();
//					for(BlackboardListener changedListener: updateSubscribers){
//						changedListener.onProcessStateChanged(status);
//					}
//				}
//				break;
//				
			case "EquipletStepsBlackBoard":
				dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status = dbObject.get("status").toString();
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStatusChanged(status);
					}
				}
				break;		
	
			default:
				break;
			}
	
		}catch(InvalidDBNamespaceException | GeneralMongoException e) {
			//Logger.log(LogLevel.ERROR, "", e);
		}
	}
	public void test(){
		updateSubscribers.get(0).OnEquipleStateChanged("1","test");
		updateSubscribers.get(0).OnEquipleModeChanged("1","mode test");
	}
}
