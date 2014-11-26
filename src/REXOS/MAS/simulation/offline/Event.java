package MAS.simulation.offline;

import java.util.LinkedList;

import MAS.util.Triple;

class Event implements Comparable<Event> {
	private double time;
	private EventType type;
	private int equiplet;
	private LinkedList<ProductStep> steps;
	private Triple<Integer, Integer, ProductStep> startData;

	/**
	 * Create event
	 * used for: done event
	 * 
	 * @param time
	 *            event time
	 * @param type
	 *            event type
	 */
	public Event(double time, EventType type) {
		this.time = time;
		this.type = type;
	}

	/**
	 * Create event
	 * used for: PRODUCT event
	 * 
	 * @param time
	 * @param type
	 * @param steps
	 */
	public Event(double time, EventType type, LinkedList<ProductStep> steps) {
		this.time = time;
		this.type = type;
		this.steps = steps;
	}

	/**
	 * Create event
	 * used for: EQUIPLET_START, EQUIPLET_BREAKDOWN, EQUIPLET_REPAIRED
	 * 
	 * 
	 * @param time
	 * @param type
	 * @param data
	 */
	public Event(double time, EventType type, Triple<Integer, Integer, ProductStep> data) {
		this.time = time;
		this.type = type;
		this.startData = data;
	}

	/**
	 * Create event
	 * used for: EQUIPLET_FINISHED equiplet finished with job
	 * 
	 * @param time
	 * @param type
	 * @param equiplet
	 */
	public Event(double time, EventType type, int equiplet) {
		this.time = time;
		this.type = type;
		this.equiplet = equiplet;
	}

	/**
	 * Implement comparable so events can be compared with each other on
	 * basis of there time and type
	 * Events are compared with time and second when equal with type
	 * Events are never equal = 0, otherwise duplicates are not possible
	 * Done events are handled before init events
	 */
	@Override
	public int compareTo(Event event) {
		return time < event.getTime() ? -1 : (time > event.getTime() ? 1 : type == event.getType() ? -1 : type.compareTo(event.getType()));
	}

	public double getTime() {
		return time;
	}

	public EventType getType() {
		return type;
	}

	public int getEquiplet() {
		return equiplet;
	}

	public LinkedList<ProductStep> getSteps() {
		return steps;
	}

	public Triple<Integer, Integer, ProductStep> getStartData() {
		return startData;
	}

	@Override
	public String toString() {
		if (type == EventType.PRODUCT) {
			return String.format("Evt[time=%.2f, type=%s, steps=%s]", time, type, steps);
		} else if (type == EventType.EQUIPLET_START) {
			return String.format("Evt[time=%.2f, type=%s, equipet=%d, product=%d, step=%s]", time, type, startData.first, startData.second, startData.third);
		} else if (type == EventType.EQUIPLET_FINISHED) {
			return String.format("Evt[time=%.2f, type=%s, equipet=%s]", time, type, equiplet);
		} else if (type == EventType.EQUIPLET_BREAKDOWN) {
			return String.format("Evt[time=%.2f, type=%s, equipet=%s]", time, type, equiplet);
		} else if (type == EventType.EQUIPLET_REPAIRED) {
			return String.format("Evt[time=%.2f, type=%s, equipet=%s]", time, type, equiplet);
		} else {
			return String.format("Evt:[time=%.2f, type=%s]", time, type);
		}
	}
}
