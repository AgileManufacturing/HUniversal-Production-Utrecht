package HAL;

import generic.Mast;

import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.bson.types.ObjectId;
import org.json.JSONObject;

import util.configuration.Configuration;
import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
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
import HAL.listeners.EquipletListener.EquipletCommandStatus;
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
public class BlackboarRosInterface extends RosInterface implements BlackboardSubscriber {
	private BlackboardClient stateBlackboardBBClient;
	private BlackboardClient hardwareStepsBBClient;
	private BlackboardClient equipletCommandsBBClient;
	
	private BasicOperationSubscription stateSubscription;
	private FieldUpdateSubscription hardwareStepStatusSubscription;
	private FieldUpdateSubscription equipletCommandStatusSubscription;
	
	private Map<ObjectId, HardwareStep> hardwareSteps;
	
	/**
	 * @throws BlackboardUpdateException 
	 * 
	 */
	public BlackboarRosInterface(HardwareAbstractionLayer hal) {
		super(hal);
		this.hardwareSteps = new HashMap<ObjectId, HardwareStep>();
		String equipletName = hal.getEquipletName();
		
		try {
			long start = System.currentTimeMillis();
			stateSubscription = new BasicOperationSubscription(MongoOperation.INSERT, this);
			
			stateBlackboardBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletState/ip/", equipletName));
			stateBlackboardBBClient.setDatabase(equipletName);
			stateBlackboardBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName));
			stateBlackboardBBClient.subscribe(stateSubscription);
			
			hardwareStepStatusSubscription = new FieldUpdateSubscription("status", this);
			hardwareStepStatusSubscription.addOperation(MongoUpdateLogOperation.SET);
		   
			hardwareStepsBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/hardwareSteps/ip/", equipletName));
			hardwareStepsBBClient.setDatabase(equipletName);
			hardwareStepsBBClient.setCollection((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName));
			hardwareStepsBBClient.subscribe(hardwareStepStatusSubscription);

			equipletCommandStatusSubscription = new FieldUpdateSubscription("status", this);
			equipletCommandStatusSubscription.addOperation(MongoUpdateLogOperation.SET);

			equipletCommandsBBClient = new BlackboardClient((String) Configuration.getProperty("rosInterface/equipletCommands/ip/", equipletName));
			equipletCommandsBBClient.setDatabase(equipletName);
			equipletCommandsBBClient.setCollection((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName));
			equipletCommandsBBClient.subscribe(equipletCommandStatusSubscription);
			long end = System.currentTimeMillis();
			System.out.println("Connecting took " + (end - start));

		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.CRITICAL, "Unable to initialize HAL.BlackBoardHandler", ex);
		}
	}
	
	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		String equipletName = hal.getEquipletName();
		DBObject dbObject;
		try{
			String collectionName = entry.getNamespace().split("\\.")[1];
			if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletState/blackboardName/", equipletName))) { 
				Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "EQ state or mode changed");
				dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					if(dbObject.containsField("state")){
						String state = dbObject.get("state").toString();
						this.onEquipletStateChanged(Mast.State.valueOf(state));
					}
					if(dbObject.containsField("mode")){
						String mode = dbObject.get("mode").toString();
						this.onEquipletModeChanged(Mast.Mode.valueOf(mode));
					}
				}
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/hardwareSteps/blackboardName/", equipletName))) {
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
			} else if(collectionName.equals((String) Configuration.getProperty("rosInterface/equipletCommands/blackboardName/", equipletName))) {
				dbObject = equipletCommandsBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null){
					String status = dbObject.get("status").toString();
					//String id = ((org.bson.types.ObjectId) dbObject.get("_id")).toString();
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "Reloading of the Equiplet has: " + status);
					
					this.onEquipletCommandStatusChanged(EquipletCommandStatus.valueOf(status));
				}
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Unknown exception occured:", ex);
		}
	}
	
	public void postHardwareStep(JSONObject hardwareStep) throws InvalidJSONException, InvalidDBNamespaceException, GeneralMongoException {
		hardwareStepsBBClient.insertDocument(hardwareStep.toString() + ", { writeConcern: { w: 1, wtimeout: 0 } }");
	}
	
	public void shutdown() {
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
