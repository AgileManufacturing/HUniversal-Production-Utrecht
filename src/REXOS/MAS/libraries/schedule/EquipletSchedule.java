package libraries.schedule;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.schedule.data_classes.*;

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
		
		//insert new freetimeslot indef
		
		
	}

	public ArrayList<FreeTimeSlot> GetFreeTimeSlots(UUID lockKey, Long duration, Long deadline) throws ScheduleAccessException, InvalidDBNamespaceException, GeneralMongoException {
		super.checkLock(lockKey);
		
		ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
		
		//filter freetimeslots on not shorter than given duration
		if (duration != null){
			
		}
		//filter results on times before the deadline
		if (deadline != null){
			
		}
		//create search query
		//DBObject filterquery = QueryBuilder.start("startTimeSlot").lessThan(deadline).get();
		BasicDBObject orderBy = new BasicDBObject("startTimeSlot", "1");
		
		BasicDBObject findquery = new BasicDBObject("$orderby", orderBy);
		
		List<DBObject> freeTimeSlotDBObjects = FreeTimeSlotBlackboard.findDocuments(findquery);
		
		for(DBObject freeTimeSlotDBObject : freeTimeSlotDBObjects){
			freeTimeSlots.add(new FreeTimeSlot((BasicDBObject) freeTimeSlotDBObject));
		}
		
		return freeTimeSlots; 
	}
	
	public void ScheduleOn(UUID lockKey, ProductStepScheduleData scheduleData) throws ScheduleAccessException {
		super.checkLock(lockKey);
		// TODO Auto-generated method stub
		
		
		
	}

}
