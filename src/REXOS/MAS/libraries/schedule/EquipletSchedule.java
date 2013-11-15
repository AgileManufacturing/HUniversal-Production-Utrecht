package libraries.schedule;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.schedule.data_classes.*;

public class EquipletSchedule extends Schedule {
	
	
	private BlackboardClient planningBlackboard;
	
	private BlackboardClient FreeTimeSlotBlackboard;
	
	private BlackboardClient RealtimeBlackboard;
	
	private String databaseName;
	
	private TimeSlotSynchronization timeSlotSynchronization;
	
	public EquipletSchedule(String scheduleHostName, int schedulePort, String databaseName, TimeSlotSynchronization timeSlotSynchronization, boolean clearSchedule) 
			throws UnknownHostException, GeneralMongoException, InvalidDBNamespaceException{
		super(scheduleHostName, schedulePort);
		this.databaseName = databaseName;
		this.timeSlotSynchronization = timeSlotSynchronization;
		
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
		long currentTimeSlot = timeSlotSynchronization.getCurrentTimeSlot();
		FreeTimeSlot freeTimeSlot = new FreeTimeSlot(currentTimeSlot, null);
		
		FreeTimeSlotBlackboard.insertDocument(freeTimeSlot.toBasicDBObject());
	}

	public EquipletFreeTimeData GetFreeTimeSlots(Long duration, Long deadline) throws InvalidDBNamespaceException, GeneralMongoException, ScheduleException {
		
		
		ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
		Long infiniteFreeTimeSlot = null;
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
			FreeTimeSlot newFreeTimeSlot = new FreeTimeSlot((BasicDBObject) freeTimeSlotDBObject); 
			if ( newFreeTimeSlot.getDuration() != null){
				freeTimeSlots.add(newFreeTimeSlot);
			}
			else{
				infiniteFreeTimeSlot = newFreeTimeSlot.getStartTimeSlot();
			}
			
		}
		if (infiniteFreeTimeSlot == null){
			throw new ScheduleException("InfiniteFreeTimeSlot not found, Schedule is full or the FreeTimeSlotBlackboard has failed");
		}
		
		EquipletFreeTimeData freeTimeData = new EquipletFreeTimeData(freeTimeSlots, infiniteFreeTimeSlot, 50.0d);
		
		return freeTimeData; 
	}
	
	public void ScheduleOn(UUID lockKey, ProductStepSchedule scheduleData) throws ScheduleAccessException {
		super.checkLock(lockKey);
		// TODO Auto-generated method stub
		
		
		
	}

}
