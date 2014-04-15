package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import HAL.exceptions.BlackboardUpdateException;
import HAL.listeners.BlackboardListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.BlackboardSubscriber;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.MongoOperation;
import libraries.blackboard_client.data_classes.OplogEntry;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription.MongoUpdateLogOperation;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import com.mongodb.DBObject;

import configuration.Configuration;
import configuration.ConfigurationFiles;


/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class BlackboardUpdated implements BlackboardSubscriber {
	
	private ArrayList<BlackboardListener> updateSubscribers;
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient modeBlackboardBBClient;
	private FieldUpdateSubscription stateSubscription;
	private FieldUpdateSubscription modeSubscription;
	private String equipletName = null;
	private String state = null;
	private String mode = null;
	private String status = null;
	private BlackboardClient equipletStepBBClient;
	private FieldUpdateSubscription statusSubscription;
	private BlackboardClient productStepBBClient;
	private BlackboardClient serviceStepBBClient;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardUpdated() throws BlackboardUpdateException{
		updateSubscribers = new ArrayList<BlackboardListener>();
			
		try {
			
			stateSubscription = new FieldUpdateSubscription("state", this);
			stateSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			modeSubscription = new FieldUpdateSubscription("mode", this);
			modeSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			equipletStepBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbIp", "EQ1"));
			equipletStepBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbName", "EQ1"));
			equipletStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "EquipletStepsBlackBoardName", "EQ1"));
			equipletStepBBClient.subscribe(statusSubscription);
			
			productStepBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbIp", "EQ1"));
			productStepBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbName", "EQ1"));
			productStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ProductStepsBlackBoardName", "EQ1"));
			productStepBBClient.subscribe(statusSubscription);
			
			serviceStepBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbIp", "EQ1"));
			serviceStepBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbName", "EQ1"));
			serviceStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ServiceStepsBlackBoardName", "EQ1"));
			serviceStepBBClient.subscribe(statusSubscription);
			
			stateBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			stateBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			stateBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			modeBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			modeBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			modeBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			modeBlackboardBBClient.subscribe(modeSubscription);

			
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException ex) {
			// TODO Auto-generated catch block
			throw new BlackboardUpdateException(ex.toString(), ex);
		}
	}
	

	/**
	 * 
	 * @param blackboardListener
	 */
	public void addOnBlackBoardUpdatedListener(BlackboardListener blackboardListener){
		updateSubscribers.add(blackboardListener);
		
	}

	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
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
						changedListener.OnEquipletStateChanged(equipletName,state);
						changedListener.OnEquipletModeChanged(equipletName, mode);
					}
				}
					
				break;
				
			case "EquipletStepsBlackBoard":
				dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status= dbObject.get("status").toString();
					System.out.println("EQ step process status changed");
					
					if (!status.equals("IN_PROGRESS")){
						for(BlackboardListener changedListener: updateSubscribers){
							changedListener.onProcessStatusChanged(status);	
						}
					}
				}
				break;
				
			case "ProductStepsBlackBoard":
				System.out.println("Product step process status changed");
				dbObject = productStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status= dbObject.get("status").toString();
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStatusChanged(status);	
					}
				}
				break;
				
			case "ServiceStepsBlackBoard":
				System.out.println("Service step process status changed");
				dbObject = serviceStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status= dbObject.get("status").toString();
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStatusChanged(status);	
					}
				}
				break;
			}
	
		}catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
	}

}
