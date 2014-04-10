package HAL;

import jade.core.AID;
import jade.core.Agent;

import java.net.UnknownHostException;
import java.util.ArrayList;

import agents.data_classes.DbData;
import HAL.listeners.BlackboardListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.BlackboardSubscriber;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.MongoOperation;
import libraries.blackboard_client.data_classes.OplogEntry;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import com.mongodb.DBObject;

import configuration.Configuration;
import configuration.ConfigurationFiles;

public class BlackboardUpdated extends Agent implements BlackboardSubscriber {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ArrayList<BlackboardListener> updateSubscribers;
	private BlackboardClient serviceStepBBClient;
	private BlackboardClient productStepBBClient;
	private BlackboardClient equipletStepBBClient;
	private BlackboardClient stateBlackboardBBClient;
	private AID equipletAgentAID;
	private DbData dbData;
	private AID serviceAgentAID;

	public BlackboardUpdated(){
		updateSubscribers = new ArrayList<BlackboardListener>();
		Object[] args = getArguments();
		if(args != null && args.length > 0) {
			dbData = (DbData) args[0];
			equipletAgentAID = (AID) args[1];
			serviceAgentAID = (AID) args[2];
		}
			
		try {
			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ServiceStepsBlackBoardName", serviceAgentAID.getLocalName()));
		
			equipletStepBBClient = new BlackboardClient(dbData.getIp());
			equipletStepBBClient.setDatabase(dbData.getName());
			equipletStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "EquipletStepsBlackBoardName", equipletAgentAID.getLocalName()));
		
			productStepBBClient = new BlackboardClient(dbData.getIp());
			productStepBBClient.setDatabase(dbData.getName());
			productStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ProductstepsBlackBoardName", equipletAgentAID.getLocalName()));
			
			
			stateBlackboardBBClient = new BlackboardClient(dbData.getIp());
			stateBlackboardBBClient.setDatabase("StateBlackboardDbName");
			stateBlackboardBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName", "stateBlackBoardName"));
			
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
		String id;
		String state;
		String mode;
		String status;
		DBObject dbObject;
		try{
			switch (entry.getNamespace().split("\\.")[1]) {
			
			
			case "equipletState":
				dbObject = stateBlackboardBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					id = dbObject.get("id").toString();
					state = dbObject.get("state").toString();
					mode = dbObject.get("mode").toString();
					Logger.log(LogLevel.DEBUG, "Ari EquipletState State set to: %s%n", state);
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.OnEquipleStateChanged(id,state);
						changedListener.OnEquipleModeChanged(id, mode);
					}
				}
					
				break;
				
			case "ProductStepsBlackboard":
				dbObject = productStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status = dbObject.get("status").toString();
					Logger.log(LogLevel.DEBUG, "Ari ProductStep status set to: %s%n", status);
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStateChanged(status);
					}
				}
				
				break;
				
			case "ServiceStepsBlackboard":
				dbObject = serviceStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status = dbObject.get("status").toString();
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStateChanged(status);
					}
				}
				break;
				
			case "EquipletStepsBlackboard":
				dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
				if(dbObject != null) {
					status = dbObject.get("status").toString();
					for(BlackboardListener changedListener: updateSubscribers){
						changedListener.onProcessStateChanged(status);
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
