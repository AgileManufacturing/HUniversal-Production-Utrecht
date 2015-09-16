package HAL;

import generic.Mast;

import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.bson.types.ObjectId;
import org.json.JSONException;
import org.json.JSONObject;

import util.configuration.Configuration;
import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.BlackboardUpdateException;
import HAL.libraries.blackboard_client.BlackboardClient;
import HAL.libraries.blackboard_client.data_classes.BasicOperationSubscription;
import HAL.libraries.blackboard_client.data_classes.BlackboardSubscriber;
import HAL.libraries.blackboard_client.data_classes.FieldUpdateSubscription;
import HAL.libraries.blackboard_client.data_classes.FieldUpdateSubscription.MongoUpdateLogOperation;
import HAL.libraries.blackboard_client.data_classes.GeneralMongoException;
import HAL.libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import HAL.libraries.blackboard_client.data_classes.InvalidJSONException;
import HAL.libraries.blackboard_client.data_classes.MongoOperation;
import HAL.libraries.blackboard_client.data_classes.OplogEntry;
import HAL.listeners.EquipletCommandListener.EquipletCommandStatus;
import HAL.listeners.ViolationListener.ViolationType;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

import com.mongodb.DBObject;


/**
 * A BlackBoardhandler handles message going to and coming back off blackboard.
 * 
 * @author Aristides Ayala Mendoza
 * @author Lars Veenendaal
 * 
 */
public class BlackboardRosInterface extends RosInterface implements BlackboardSubscriber {
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient hardwareStepsBBClient;
	private BlackboardClient equipletCommandsBBClient;
	private BlackboardClient violationsBBClient;
	
	private BasicOperationSubscription stateSubscription;
	private FieldUpdateSubscription hardwareStepStatusSubscription;
	private FieldUpdateSubscription equipletCommandStatusSubscription;
	private BasicOperationSubscription violationsSubscription;
	
	private Map<ObjectId, HardwareStep> hardwareSteps;
	
	AbstractHardwareAbstractionLayer hal;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboardRosInterface(AbstractHardwareAbstractionLayer hal) {
		super(hal.getModuleFactory());
		this.hal = hal;
		this.hardwareSteps = new HashMap<ObjectId, HardwareStep>();
		String equipletName = hal.getEquipletName();
		String databaseName = equipletName;
		if(hal.isShadow() == true) databaseName = "shadow_" + databaseName;
		
		try {
			stateSubscription = new BasicOperationSubscription(MongoOperation.INSERT, this);
			
			stateBlackboardBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletState/ip/", equipletName));
			stateBlackboardBBClient.setDatabase(databaseName);
			stateBlackboardBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			hardwareStepStatusSubscription = new FieldUpdateSubscription("status", this);
			hardwareStepStatusSubscription.addOperation(MongoUpdateLogOperation.SET);
		   
			hardwareStepsBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/hardwareSteps/ip/", equipletName));
			hardwareStepsBBClient.setDatabase(databaseName);
			hardwareStepsBBClient.setCollection((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName));
			hardwareStepsBBClient.subscribe(hardwareStepStatusSubscription);

			equipletCommandStatusSubscription = new FieldUpdateSubscription("status", this);
			equipletCommandStatusSubscription.addOperation(MongoUpdateLogOperation.SET);

			equipletCommandsBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletCommands/ip/", equipletName));
			equipletCommandsBBClient.setDatabase(databaseName);
			equipletCommandsBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName));
			equipletCommandsBBClient.subscribe(equipletCommandStatusSubscription);
			
			violationsSubscription = new BasicOperationSubscription(MongoOperation.INSERT, this);
			
			violationsBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/violations/ip/", equipletName));
			violationsBBClient.setDatabase(databaseName);
			violationsBBClient.setCollection((String) Configuration.getProperty("rosInterface/violations/blackboardName/", equipletName));
			violationsBBClient.subscribe(violationsSubscription);
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Unable to initialize HAL.BlackBoardHandler", ex);
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
			if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", hal.getEquipletName()))) { 
				Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "State or mode changed");
				dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					if(dbObject.containsField("state")){
						String state = dbObject.get("state").toString();
						if(dbObject.containsField("moduleIdentifier")) {
							String moduleIdentifierString = dbObject.get("moduleIdentifier").toString();
							JSONObject moduleIdentifierJson = new JSONObject(moduleIdentifierString);
							ModuleIdentifier identifier = new ModuleIdentifier(
									moduleIdentifierJson.getString("manufacturer"),
									moduleIdentifierJson.getString("typeNumber"),
									moduleIdentifierJson.getString("serialNumber"));
							this.onModuleStateChanged(identifier, Mast.State.valueOf(state));
						} else {
							this.onEquipletStateChanged(Mast.State.valueOf(state));
						}
					}
					if(dbObject.containsField("mode")){
						String mode = dbObject.get("mode").toString();
						if(dbObject.containsField("moduleIdentifier")) {
							String moduleIdentifierString = dbObject.get("moduleIdentifier").toString();
							JSONObject moduleIdentifierJson = new JSONObject(moduleIdentifierString);
							ModuleIdentifier identifier = new ModuleIdentifier(
									moduleIdentifierJson.getString("manufacturer"),
									moduleIdentifierJson.getString("typeNumber"),
									moduleIdentifierJson.getString("serialNumber"));
							this.onModuleModeChanged(identifier, Mast.Mode.valueOf(mode));
						} else {
							this.onEquipletModeChanged(Mast.Mode.valueOf(mode));
						}
					}
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", hal.getEquipletName()))) {
				DBObject document = entry.getModifiedDocumentPart();
				ObjectId id = entry.getTargetObjectId();
				String status = document.get("status").toString();
				
				Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "EQ step process status changed: " + status + " " + id);
				
				HardwareStep step;
				synchronized (hardwareSteps) {
					step = hardwareSteps.get(id);
					
					step.setStatus(HardwareStepStatus.valueOf(status));
					if(step.getStatus() == HardwareStepStatus.DONE || step.getStatus() == HardwareStepStatus.FAILED) {
						// we are done with this step
						hardwareSteps.remove(id);
					}
				}
				this.onHardwareStepStatusChanged(step);
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", hal.getEquipletName()))) {
				dbObject = equipletCommandsBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null){
					String status = dbObject.get("status").toString();
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "Equiplet command status: " + status);
					
					this.onEquipletCommandStatusChanged(EquipletCommandStatus.valueOf(status));
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/violations/blackboardName/", hal.getEquipletName()))) {
				dbObject = violationsBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null){
					ViolationType type = ViolationType.valueOf(dbObject.get("type").toString());
					String message = dbObject.get("message").toString();
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "Violation occured: " + type + " " + message);
					
					this.onViolationOccured(type, message);
				}
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException | JSONException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Unknown exception occured:", ex);
		}
	}
	
	public void postHardwareStep(JSONObject hardwareStep) throws InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		hardwareStepsBBClient.insertDocument(hardwareStep.toString() + ", { writeConcern: { w: 1, wtimeout: 0 } }");
	}
	
	public void shutdown() {
		violationsBBClient.close();
		equipletCommandsBBClient.close();
		hardwareStepsBBClient.close();
		stateBlackboardBBClient.close();
	}
	
	@Override
	public void postHardwareStep(HardwareStep hardwareStep) {
		try {
			synchronized (hardwareSteps) {
				long start = System.currentTimeMillis();
				ObjectId id = hardwareStepsBBClient.insertDocument(hardwareStep.toJSON().toString() + ", { writeConcern: { w: 1, wtimeout: 0 } }");
				long end = System.currentTimeMillis();
				System.out.println(end - start);
				hardwareSteps.put(id, hardwareStep);
			}
		} catch (InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting hardware step failed", ex);
			throw new RuntimeException(ex);
		}
	}

	@Override
	public void postEquipletCommand(JSONObject equipletCommand) {
		try {
			equipletCommandsBBClient.insertDocument(equipletCommand.toString() + ", { writeConcern: { w: 1, wtimeout: 0 } }");
		} catch (InvalidJSONException | InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Posting command failed", ex);
			throw new RuntimeException(ex);
		}
	}
}
