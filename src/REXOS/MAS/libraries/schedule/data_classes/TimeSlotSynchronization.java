package libraries.schedule.data_classes;

public class TimeSlotSynchronization {
	
	private long firstTimeSlot;
	
	private long timeSlotLength;
	
	public TimeSlotSynchronization(long firstTimeSlot, long timeSlotLength){
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;		
	}
	
	public long getFirstTimeSlot(){
		return firstTimeSlot;
	}
	
	public long getTimeSlotLength(){
		return timeSlotLength;
	}
	
	public long getCurrentTimeSlot(){
		return (System.currentTimeMillis() - firstTimeSlot) / timeSlotLength;
	}
}
