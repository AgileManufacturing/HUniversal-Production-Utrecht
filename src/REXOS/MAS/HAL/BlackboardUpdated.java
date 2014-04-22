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
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardUpdated() throws BlackboardUpdateException{
		updateSubscribers = new ArrayList<BlackboardListener>();
			
		try {
			
			stateSubscription = new FieldUpdateSubscription("state", this);
			stateSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			modeSubscription = new FieldUpdateSubscription("mode", this);
			modeSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			
			stateBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			stateBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			stateBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			modeBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			modeBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			modeBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			modeBlackboardBBClient.subscribe(modeSubscription);

			
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException e) {
			// TODO Auto-generated catch block
			throw new BlackboardUpdateException(e.toString());
		}
	}
	

	/**
	 * 
	 * @param blackboardListener
	 */
	public void setOnBlackBoardUpdatedListener(BlackboardListener blackboardListener){
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
				

			default:
				break;
			}
	
		}catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
	}

}