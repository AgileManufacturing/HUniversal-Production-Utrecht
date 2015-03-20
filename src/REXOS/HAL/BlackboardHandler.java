package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

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
import HAL.listeners.BlackboardEquipletListener;
import HAL.listeners.BlackboardModuleListener;
import HAL.listeners.BlackboardProcessListener;
import HAL.steps.HardwareStep.HardwareStepStatus;

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
public class BlackboardHandler implements BlackboardSubscriber {

	private ArrayList<BlackboardModuleListener> moduleSubscribers;
	private ArrayList<BlackboardEquipletListener> equipletSubscribers;
	private ArrayList<BlackboardProcessListener> processSubscribers;
	
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient modeBlackboardBBClient;
	private BlackboardClient equipletStepBBClient;
	private BlackboardClient ReloadEquipletBBClient;
	
	private FieldUpdateSubscription stateSubscription;
	private FieldUpdateSubscription modeSubscription;
	private FieldUpdateSubscription statusSubscription;
	private FieldUpdateSubscription reloadSubscription;
	
	private String state = null;
	private String mode = null;
	private String equipletName;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardHandler(String equipletName) throws BlackboardUpdateException{
		this.equipletName = equipletName;
		moduleSubscribers = new ArrayList<BlackboardModuleListener>();
		equipletSubscribers = new ArrayList<BlackboardEquipletListener>();
		processSubscribers = new ArrayList<BlackboardProcessListener>();
		
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
		   
			equipletStepBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/hardwareSteps/ip/", equipletName));
			equipletStepBBClient.setDatabase((String) Configuration.getProperty("rosInterface/hardwareSteps/databaseName/", equipletName));
			equipletStepBBClient.setCollection((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName));
			equipletStepBBClient.subscribe(statusSubscription);

			reloadSubscription = new FieldUpdateSubscription("reload", this);
			reloadSubscription.addOperation(MongoUpdateLogOperation.SET);

			ReloadEquipletBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletCommands/ip/", equipletName));
			ReloadEquipletBBClient.setDatabase((String) Configuration.getProperty("rosInterface/equipletCommands/databaseName/", equipletName));
			ReloadEquipletBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName));
			ReloadEquipletBBClient.subscribe(reloadSubscription);

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
	public void addBlackboardProcessListener(BlackboardProcessListener blackboardListener){
		processSubscribers.add(blackboardListener);
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
	 * 
	 * @param blackboardListener
	 */
	public void removeBlackboardProcessListener(BlackboardProcessListener blackboardListener){
		processSubscribers.remove(blackboardListener);
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
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName))) {
				dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
				
				if(dbObject != null) {
					String status = dbObject.get("status").toString();
					String id = ((org.bson.types.ObjectId) dbObject.get("_id")).toString();
					Logger.log(LogSection.HAL_BLACKBOARD, LogLevel.DEBUG, "EQ step process status changed: " + status + " " + id);
					
					for(BlackboardProcessListener listener: processSubscribers) {
						listener.onProcessStatusChanged(HardwareStepStatus.valueOf(status), id); 
					}
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName))) {
			    /**
			     * ReloadEquiplet - W.I.P (Lars Veenendaal)
			     */
				dbObject = ReloadEquipletBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null){
					String status = dbObject.get("ReloadSucces").toString();
					//String id = ((org.bson.types.ObjectId) dbObject.get("_id")).toString();
					Logger.log(LogSection.HAL_BLACKBOARD, LogLevel.DEBUG, "Reloading of the Equiplet has: " + status);
					for(BlackboardEquipletListener listener: equipletSubscribers) {
						listener.onReloadEquiplet(status); 
					}
				}
			}
		}catch(InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_BLACKBOARD, LogLevel.ERROR, "Unknown exception occured:", ex);
		}
	}
	
	public void postHardwareStep(JSONObject hardwareStep) throws InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		equipletStepBBClient.insertDocument(hardwareStep.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
	}
	
	/**
	 * [postReloadEquiplet - This method posts to blackboard that a equiplet has to reload. - UNTESTED W.I.P (Lars Veenendaal)]
	 * @throws JSONException
	 * @throws InvalidJSONException
	 * @throws InvalidDBNamespaceException
	 * @throws GeneralMongoException
	 */
	public void postReloadEquiplet() throws JSONException, InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		JSONObject reloadEQ = new JSONObject("{\"reloadEquiplet\":RELOAD_ALL_MODULES}");
		equipletStepBBClient.insertDocument(reloadEQ.toString() + ", { writeConcern: { w: 2, wtimeout: 0 } }");
	}
	
	public void postShutdownEquiplet() throws JSONException, InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException{
		JSONObject shutdownEQ = new JSONObject("{'shutdownEquiplet':SHUTDOWN_ALL_MODULES}");
		stateBlackboardBBClient.insertDocument(shutdownEQ.toString());
	}
}
