package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import util.configuration.Configuration;
import util.configuration.ConfigurationFiles;
import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.exceptions.BlackboardUpdateException;
import HAL.libraries.blackboard_client.BlackboardClient;
import HAL.libraries.blackboard_client.data_classes.BlackboardSubscriber;
import HAL.libraries.blackboard_client.data_classes.FieldUpdateSubscription;
import HAL.libraries.blackboard_client.data_classes.FieldUpdateSubscription.MongoUpdateLogOperation;
import HAL.libraries.blackboard_client.data_classes.GeneralMongoException;
import HAL.libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import HAL.libraries.blackboard_client.data_classes.InvalidJSONException;
import HAL.libraries.blackboard_client.data_classes.MongoOperation;
import HAL.libraries.blackboard_client.data_classes.OplogEntry;
import HAL.listeners.BlackboardEquipletListener;
import HAL.listeners.BlackboardModuleListener;

import org.json.JSONObject;
import com.mongodb.DBObject;


/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class BlackboardHandler implements BlackboardSubscriber {

	private ArrayList<BlackboardModuleListener> moduleSubscribers;
	private ArrayList<BlackboardEquipletListener> equipletSubscribers;
	
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient modeBlackboardBBClient;
	private BlackboardClient equipletStepBBClient;
	
	private FieldUpdateSubscription stateSubscription;
	private FieldUpdateSubscription modeSubscription;
	private FieldUpdateSubscription statusSubscription;
	
	private String state = null;
	private String mode = null;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardHandler(String equipletName) throws BlackboardUpdateException{
		moduleSubscribers = new ArrayList<BlackboardModuleListener>();
		equipletSubscribers = new ArrayList<BlackboardEquipletListener>();
			
		try {
			stateSubscription = new FieldUpdateSubscription("state", this);
			stateSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			stateBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			stateBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			stateBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			
			modeSubscription = new FieldUpdateSubscription("mode", this);
			modeSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			modeBlackboardBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
			modeBlackboardBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			modeBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));
			modeBlackboardBBClient.subscribe(modeSubscription);
			
			
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
		   
			equipletStepBBClient = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbIp", equipletName));
			equipletStepBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbName", equipletName));
			equipletStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "EquipletStepsBlackBoardName", equipletName));
			equipletStepBBClient.subscribe(statusSubscription);
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException ex) {
			throw new BlackboardUpdateException("Unable to initialize HAL.BlackBoardHandler", ex);
		}
	}
	

	/**
	 * 
	 * @param blackboardListener
	 */
	public void addBlackboardModuleListener(BlackboardModuleListener blackboardListener){
		moduleSubscribers.add(blackboardListener);
	}
	/**
	 * 
	 * @param blackboardListener
	 */
	public void addBlackboardEquipletListener(BlackboardEquipletListener blackboardListener){
		equipletSubscribers.add(blackboardListener);
	}
	

	/**
	 * 
	 * @param blackboardListener
	 */
	public void removeBlackboardEquipletListener(BlackboardEquipletListener blackboardListener){
		equipletSubscribers.remove(blackboardListener);
	}
	/**
	 * 
	 * @param blackboardListener
	 */
	public void removeBlackboardModuleListener(BlackboardModuleListener blackboardListener){
		moduleSubscribers.remove(blackboardListener);
	}
	
	

	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		DBObject dbObject;
		try{
			switch (entry.getNamespace().split("\\.")[1]) {
				case "equipletState":
					Logger.log(LogSection.HAL_BLACKBOARD, LogLevel.DEBUG, "EQ state or mode changed");
					dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
					if(dbObject != null) {
						if(dbObject.containsField("state")){
							state = dbObject.get("state").toString();
						}
						if(dbObject.containsField("mode")){
							mode = dbObject.get("mode").toString();
						}	
						for(BlackboardEquipletListener listener: equipletSubscribers){
							listener.onEquipletStateChanged(state);
							listener.onEquipletModeChanged(mode);
						}
					}
					break;
				case "EquipletStepsBlackBoard":
					dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
					if(dbObject != null) {
						String status = dbObject.get("status").toString();
						Logger.log(LogSection.HAL_BLACKBOARD, LogLevel.DEBUG, "EQ step process status changed");
						
						for(BlackboardModuleListener listener: moduleSubscribers) {
							listener.onProcessStatusChanged(status); 
						}
					}
				    break;
				default:
					break;
			}
		}catch(InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogLevel.ERROR, "Unknown exception occured:", ex);
		}
	}
	
	public void postHardwareStep(JSONObject hardwareStep) throws InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		equipletStepBBClient.insertDocument(hardwareStep.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
	}

}
