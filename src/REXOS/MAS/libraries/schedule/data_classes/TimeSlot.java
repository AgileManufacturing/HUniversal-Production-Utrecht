package libraries.schedule.data_classes;

public class TimeSlot {

	/**
	 * @var long startTimeSlot 
	 * 		The start time in timeslots.
	 **/
	private long startTimeSlot;

	/**
	 * @var Long duration
	 * 		the duration of this freetimeslot.
	 * 		Can be null! if it is, the starttime of this freetimeslot 
	 *		will be the the first timeslot after the last scheduled step.
	 */
	private Long duration;
	
	public TimeSlot(){
	}
	
	public TimeSlot(long startTimeSlot, Long duration){
		this.startTimeSlot = startTimeSlot;
		this.duration = duration;
	}
	
	public long getStartTimeSlot(){
		return startTimeSlot;
	}
	
	public Long getDuration(){
		return duration;
	}
	
	public void setStartTimeSlot(long startTimeSlot){
		this.startTimeSlot = startTimeSlot;
	}
	
	public void setDuration(Long duration){
		this.duration = duration;
	}
}
