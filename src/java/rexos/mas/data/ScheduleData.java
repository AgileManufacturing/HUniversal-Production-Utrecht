package rexos.mas.data;

import java.io.Serializable;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Instances of this class contain schedule data with the start time, duration and the deadline of a <code>ProductionStep</code> in timeslots.
 * 
 * @author Peter Bonnema
 *
 */
public class ScheduleData implements MongoSaveable, Serializable {
	private static final long serialVersionUID = 8380622365668923276L;
	private int startTime;
	private int duration;
	private int deadline;
	
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
	public ScheduleData(int startTime, int duration, int deadline) {
		this.startTime = startTime;
		this.duration = duration;
		this.deadline = deadline;
	}

	/* (non-Javadoc)
	 * @see newDataClasses.DBSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("startTime", startTime)
				.add("duration", duration)
				.add("deadline", deadline).get();
	}

	/* (non-Javadoc)
	 * @see newDataClasses.DBSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		this.startTime = object.getInt("startTime", -1);
		this.duration = object.getInt("duration", -1);
		this.deadline = object.getInt("deadline", -1);
	}

	/**
	 * @return the start time
	 */
	public int getStartTime() {
		return startTime;
	}

	/**
	 * @param startTime the start time to set
	 */
	public void setStartTime(int startTime) {
		this.startTime = startTime;
	}

	/**
	 * @return the duration
	 */
	public int getDuration() {
		return duration;
	}

	/**
	 * @param duration the duration to set
	 */
	public void setDuration(int duration) {
		this.duration = duration;
	}

	/**
	 * @return the deadline
	 */
	public int getDeadline() {
		return deadline;
	}

	/**
	 * @param deadline the deadline to set
	 */
	public void setDeadline(int deadline) {
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
