package equipletAgent;

/**
 * Instances of this class contain schedule data with the start time, duration and the deadline of a production step in timeslots.
 * 
 * @author Peter Bonnema
 *
 */
public class ScheduleData {
	/**
	 * When the production step execution should start.
	 */
	private long startTime;
	/**
	 * How long it will take to complete the step
	 */
	private long duration;
	/**
	 * When the production step should be finished.
	 */
	private long deadline;
	
	/**
	 * Creates a new ScheduleData with the specified start time, duration and deadline.
	 * 
	 * @param startTime
	 * @param duration
	 * @param deadline
	 */
	public ScheduleData(long startTime, long duration, long deadline) {
		this.startTime = startTime;
		this.duration = duration;
		this.deadline = deadline;
	}

	/**
	 * @return the startTime
	 */
	public long getStartTime() {
		return startTime;
	}

	/**
	 * @param startTime the startTime to set
	 */
	public void setStartTime(long startTime) {
		this.startTime = startTime;
	}

	/**
	 * @return the duration
	 */
	public long getDuration() {
		return duration;
	}

	/**
	 * @param duration the duration to set
	 */
	public void setDuration(long duration) {
		this.duration = duration;
	}

	/**
	 * @return the deadline
	 */
	public long getDeadline() {
		return deadline;
	}

	/**
	 * @param deadline the deadline to set
	 */
	public void setDeadline(long deadline) {
		this.deadline = deadline;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format(
				"ScheduleData [startTime=%s, duration=%s, deadline=%s]",
				startTime, duration, deadline);
	}
}
