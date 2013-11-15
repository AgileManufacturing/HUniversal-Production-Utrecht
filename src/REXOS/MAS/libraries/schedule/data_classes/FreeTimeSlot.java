package libraries.schedule.data_classes;

import libraries.blackboard_client.data_classes.MongoSaveable;

import com.mongodb.BasicDBObject;

public class FreeTimeSlot implements MongoSaveable{
	
	private Long startTimeSlot;
	
	private Long duration;
	
	public FreeTimeSlot(BasicDBObject dbObject){
		fromBasicDBObject(dbObject);
	}
	
	public FreeTimeSlot(Long startTimeSlot, Long duration){
		this.startTimeSlot = startTimeSlot;
		this.duration = duration;
	}

	public Long getStartTimeSlot(){
		return startTimeSlot;
	}
	
	public Long getDuration(){
		return duration;
	}
	
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject freeTimeSlotDBObject = new BasicDBObject();
		freeTimeSlotDBObject.put("startTimeSlot", startTimeSlot);
		freeTimeSlotDBObject.put("duration", duration);
		return freeTimeSlotDBObject;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object)
			throws IllegalArgumentException {
		
		BasicDBObject dbObjectCopy= (BasicDBObject) object.copy();
		
		this.startTimeSlot = dbObjectCopy.getLong("statTimeSlot");
		dbObjectCopy.remove("startTimeSlot");
		this.duration = dbObjectCopy.getLong("duration");
		dbObjectCopy.remove("duration");

		if (!dbObjectCopy.isEmpty()){
			throw new IllegalArgumentException("FreeTimeSlot not parsed completely");
		}
		
	}
}