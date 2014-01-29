package simulation.data;

public class FreeTimeSlot {

	private TimeSlot timeSlot;

	public FreeTimeSlot(long startTimeSlot, Long duration){
		timeSlot = new TimeSlot(startTimeSlot, duration);
	}
	
	public TimeSlot getTimeSlot() {
		return timeSlot;
	}

	public void setTimeSlot(TimeSlot timeSlot) {
		this.timeSlot = timeSlot;
	}
	
	
	
}
