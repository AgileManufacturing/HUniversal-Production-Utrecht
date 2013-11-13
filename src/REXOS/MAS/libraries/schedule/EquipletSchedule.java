package libraries.schedule;

import java.net.UnknownHostException;
import java.util.UUID;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.schedule.data_classes.ProductStepScheduleData;
import libraries.schedule.data_classes.Schedule;
import libraries.schedule.data_classes.ScheduleAccessException;

public class EquipletSchedule extends Schedule {
	
	
	
	private BlackboardClient planningBlackboard;
	
	private BlackboardClient FreeTimeSlotBlackboard;
	
	private BlackboardClient RealtimeBlackboard;
	
	private String databaseName;
	
	public EquipletSchedule(String scheduleHostName, int schedulePort, String databaseName, boolean clearSchedule) 
			throws UnknownHostException, GeneralMongoException, InvalidDBNamespaceException{
		super(scheduleHostName, schedulePort);
		this.databaseName = databaseName;
		
		Setup();
		
		if (clearSchedule){
			ClearSchedule();
		}
	}
	
	private void Setup() throws UnknownHostException, GeneralMongoException, InvalidDBNamespaceException{
		planningBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		FreeTimeSlotBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		RealtimeBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		
		planningBlackboard.setDatabase(databaseName);
		FreeTimeSlotBlackboard.setDatabase(databaseName);
		RealtimeBlackboard.setDatabase(databaseName);
		
		planningBlackboard.setCollection(PLANNING_NAME + "Blackboard");
		FreeTimeSlotBlackboard.setCollection(FREETIMESLOT_NAME + "Blackboard");
		RealtimeBlackboard.setCollection(REALTIMESCHEDULE_NAME + "Blackboard");
	}
	
	private void ClearSchedule() throws InvalidDBNamespaceException, GeneralMongoException{
		planningBlackboard.removeDocuments(new BasicDBObject());
		FreeTimeSlotBlackboard.removeDocuments(new BasicDBObject());
		RealtimeBlackboard.removeDocuments(new BasicDBObject());
	}

	@Override
	public void GetFreeTimeSlots(UUID lockKey) throws ScheduleAccessException {
		super.GetFreeTimeSlots(lockKey);
		// TODO Auto-generated method stub
	}

	@Override
	public void ScheduleOn(UUID lockKey, ProductStepScheduleData scheduleData) throws ScheduleAccessException {
		super.ScheduleOn(lockKey, scheduleData);
		// TODO Auto-generated method stub
		
	}

}
