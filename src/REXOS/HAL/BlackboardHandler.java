package HAL;

import generic.Mast;
import generic.Mast.Mode;
import generic.Mast.State;

import java.net.UnknownHostException;
import java.util.Map;

import util.configuration.Configuration;
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
import HAL.listeners.EquipletListener.EquipletReloadStatus;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

import org.bson.types.ObjectId;
import org.json.JSONObject;
import org.json.JSONException;

import com.mongodb.DBObject;


/**
 * A BlackBoardhandler handles message going to and coming back off blackboard.
 * 
 * @author Aristides Ayala Mendoza
 * @author Lars Veenendaal
 * 
 */
public class BlackboardHandler extends RosInterface implements BlackboardSubscriber {
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient modeBlackboardBBClient;
	private BlackboardClient hardwareStepBBClient;
	private BlackboardClient equipletCommandBBClient;
	
	private FieldUpdateSubscription stateSubscription;
	private FieldUpdateSubscription modeSubscription;
	private FieldUpdateSubscription statusSubscription;
	private FieldUpdateSubscription reloadSubscription;
	
	private String state = null;
	private String mode = null;
	private String equipletName;
	
	private Map<ObjectId, HardwareStep> hardwareSteps;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardHandler(String equipletName) throws BlackboardUpdateException{
		this.equipletName = equipletName;
		
		try {
			stateSubscription = new FieldUpdateSubscription("state", this);
			stateSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			stateBlackboardBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletState/ip/", equipletName));
			stateBlackboardBBClient.setDatabase((String) Configuration.getProperty("rosInterface/equipletState/databaseName/", equipletName));
			stateBlackboardBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			
			modeSubscription = new FieldUpdateSubscription("mode", this);
			modeSubscription.addOperation(MongoUpdateLogOperation.SET);
			
			modeBlackboardBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletState/ip/", equipletName));
			modeBlackboardBBClient.setDatabase((String) Configuration.getProperty("rosInterface/equipletState/databaseName/", equipletName));
			modeBlackboardBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName));
			modeBlackboardBBClient.subscribe(modeSubscription);
			
			
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
		   
			hardwareStepBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/hardwareSteps/ip/", equipletName));
			hardwareStepBBClient.setDatabase((String) Configuration.getProperty("rosInterface/hardwareSteps/databaseName/", equipletName));
			hardwareStepBBClient.setCollection((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName));
			hardwareStepBBClient.subscribe(statusSubscription);

			reloadSubscription = new FieldUpdateSubscription("reload", this);
			reloadSubscription.addOperation(MongoUpdateLogOperation.SET);

			equipletCommandBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletCommands/ip/", equipletName));
			equipletCommandBBClient.setDatabase((String) Configuration.getProperty("rosInterface/equipletCommands/databaseName/", equipletName));
			equipletCommandBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName));
			equipletCommandBBClient.subscribe(reloadSubscription);

		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException ex) {
			throw new BlackboardUpdateException("Unable to initialize HAL.BlackBoardHandler", ex);
		}
	}
	
	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		DBObject dbObject;
		try{
			String collectionName = entry.getNamespace().split("\\.")[1];
			if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName))) { 
				Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "EQ state or mode changed");
				dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					if(dbObject.containsField("state")){
						state = dbObject.get("state").toString();
						onEquipletStateChanged(Mast.State.valueOf(state));
					}
					if(dbObject.containsField("mode")){
						mode = dbObject.get("mode").toString();
						onEquipletModeChanged(Mast.Mode.valueOf(mode));
					}
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName))) {
				dbObject = hardwareStepBBClient.findDocumentById(entry.getTargetObjectId());
				
				if(dbObject != null) {
					String status = dbObject.get("status").toString();
					String id = ((org.bson.types.ObjectId) dbObject.get("_id")).toString();
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "EQ step process status changed: " + status + " " + id);
					
					
					onHardwareStepStatusChanged(hardwareSteps.get(id), HardwareStepStatus.valueOf(status));
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName))) {
				dbObject = equipletCommandBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null){
					String status = dbObject.get("status").toString();
					//String id = ((org.bson.types.ObjectId) dbObject.get("_id")).toString();
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "Reloading of the Equiplet has: " + status);
					
					onEquipletReloadStatusChanged(EquipletReloadStatus.valueOf(status));
				}
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Unknown exception occured:", ex);
		}
	}
	
	public void postHardwareStep(JSONObject hardwareStep) throws InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		hardwareStepBBClient.insertDocument(hardwareStep.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
	}
	
	public void shutdown() {
		equipletCommandBBClient.close();
		hardwareStepBBClient.close();
		modeBlackboardBBClient.close();
		stateBlackboardBBClient.close();
	}
	
	@Override
	public void postHardwareStep(HardwareStep hardwareStep) {
		try {
			ObjectId id = hardwareStepBBClient.insertDocument(hardwareStep.toJSON().toString());
			hardwareSteps.put(id, hardwareStep);
		} catch (InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting hardware step failed", ex);
			throw new RuntimeException(ex);
		}
	}

	@Override
	public void postStateChange(State desiredState) {
		try {
			JSONObject reloadEQ;
			reloadEQ = new JSONObject();
			reloadEQ.put("command", "CHANGE_STATE");
			reloadEQ.put("desiredMode", desiredState.toString());
			equipletCommandBBClient.insertDocument(reloadEQ.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
		} catch (JSONException | InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting command failed", ex);
			throw new RuntimeException(ex);
		}
	}

	@Override
	public void postModeChange(Mode desiredMode) {
		try {
			JSONObject reloadEQ;
			reloadEQ = new JSONObject();
			reloadEQ.put("command", "CHANGE_MODE");
			reloadEQ.put("desiredMode", desiredMode.toString());
			equipletCommandBBClient.insertDocument(reloadEQ.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
		} catch (JSONException | InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting command failed", ex);
			throw new RuntimeException(ex);
		}
	}

	@Override
	public void postReloadRosModules() {
		try {
			JSONObject reloadEQ;
			reloadEQ = new JSONObject();
			reloadEQ.put("command", "RELOAD_ALL_MODULES");
			equipletCommandBBClient.insertDocument(reloadEQ.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
		} catch (JSONException | InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting command failed", ex);
			throw new RuntimeException(ex);
		}
	}
}
