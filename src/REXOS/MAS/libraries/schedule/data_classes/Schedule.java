package libraries.schedule.data_classes;

import java.util.UUID;

public abstract class Schedule{
	
	private final String PLANNING_NAME= "Planning";
	private final String FREETIMESLOT_NAME= "FreeTimeSlot";
	private final String REALTIMESCHEDULE_NAME = "RealtimeSchedule";
	
	protected String scheduleHostName;
	protected int schedulePort;
		
	private ScheduleLock scheduleLock;
	
	
	protected Schedule( String scheduleHostName, int schedulePort){
		this.scheduleHostName = scheduleHostName;
		this.schedulePort = schedulePort;
		
		this.scheduleLock = new ScheduleLock();
	}
	
	protected void GetFreeTimeSlots(UUID lockKey) throws ScheduleAccessException{
		if (! scheduleLock.isCurrentOwner(lockKey)){
			throw new ScheduleAccessException("not owner of lock");
		}
	}
	
	protected void ScheduleOn(UUID lockKey) throws ScheduleAccessException{
		if (! scheduleLock.isCurrentOwner(lockKey)){
			throw new ScheduleAccessException("not owner of lock");
		}
	}
	
	public UUID getScheduleLock(){
		return scheduleLock.acquireScheduleLock();
	}
	
	public boolean releaseScheduleLock(UUID key){
		return scheduleLock.releaseLock(key);
	}
	
	
}
