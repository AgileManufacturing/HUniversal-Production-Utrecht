package rexos.mas.data;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Instances of this class contain schedule data with the start time, duration and the deadline of a <code>ProductionStep</code> in timeslots.
 * 
 * @author Peter Bonnema
 *
 */
public class ScheduleData implements IMongoSaveable {
	private long startTime;
	private long duration;
	private long deadline;
	
	/**
	 * Creates a new <code>ScheduleData</code> leaving <code>startTime</code>, <code>duration</code> and <code>deadline</code> uninitialized.
	 */
	public ScheduleData() {
	}
	
	/**
	 * @param object
	 */
	public ScheduleData(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * Creates a new <code>ScheduleData</code> with the specified start time, duration and deadline.
	 * 
	 * @param startTime When execution of the <code>ProductionStep</code> with this <code>ScheduleData</code> should start.
	 * @param duration How long it will take to complete the <code>ProductionStep</code>.
	 * @param deadline The <code>ProductionStep</code> with this <code>ScheduleData</code> should be finished after this timeslot.
	 */
	public ScheduleData(long startTime, long duration, long deadline) {
		this.startTime = startTime;
		this.duration = duration;
		this.deadline = deadline;
	}

	/**
	 * @return the start time
	 */
	public long getStartTime() {
		return startTime;
	}

	/**
	 * @param startTime the start time to set
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

	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (int) (deadline ^ (deadline >>> 32));
		result = prime * result + (int) (duration ^ (duration >>> 32));
		result = prime * result + (int) (startTime ^ (startTime >>> 32));
		return result;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ScheduleData other = (ScheduleData) obj;
		if (deadline != other.deadline)
			return false;
		if (duration != other.duration)
			return false;
		if (startTime != other.startTime)
			return false;
		return true;
	}

	/* (non-Javadoc)
	 * @see newDataClasses.IMongoSaveable#ToBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("startTime", startTime)
				.add("duration", duration)
				.add("deadline", deadline).get();
	}
	
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		this.startTime = object.getLong("startTime");
		this.duration = object.getLong("duration");
		this.deadline = object.getLong("deadline");
	}
}
