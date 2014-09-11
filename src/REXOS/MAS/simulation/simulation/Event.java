package simulation.simulation;

import simulation.util.Tick;

class Event implements Comparable<Event> {

	private Tick time;
	private EventType type;
	private String product;
	private String equiplet;

	/**
	 * Create event
	 * used for: DONE and PRODUCT event
	 * 
	 * @param time
	 * @param type
	 * @param steps
	 */
	public Event(Tick time, EventType type) {
		this.time = time;
		this.type = type;
	}

	/**
	 * Create event
	 * used for: START
	 * 
	 * @param time
	 * @param type
	 * @param product
	 *            name
	 * @param equiplet
	 *            name
	 */
	public Event(Tick time, EventType type, String product, String equiplet) {
		this.time = time;
		this.type = type;
		this.product = product;
		this.equiplet = equiplet;
	}

	/**
	 * Create event
	 * used for: FINISHED, BREAKDOWN and REPAIRED event
	 * 
	 * @param time
	 * @param type
	 * @param equiplet
	 *            name
	 */
	public Event(Tick time, EventType type, String equiplet) {
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
		return time.lessThan(event.getTime())  ? -1 : (time.greaterThan(event.getTime()) ? 1 : type == event.getType() ? -1 : type.compareTo(event.getType()));
	}

	public Tick getTime() {
		return time;
	}

	public EventType getType() {
		return type;
	}

	public String getProduct() {
		return product;
	}

	public String getEquiplet() {
		return equiplet;
	}

	@Override
	public String toString() {
		if (type == EventType.PRODUCT || type == EventType.DONE) {
			return String.format("Evt:[time=%s, type=%s]", time, type);
		} else if (type == EventType.ARRIVED) {
			return String.format("Evt:[time=%s, type=%s, product=%s, equipet=%s]", time, type, product, equiplet);
		} else {
			return String.format("Evt:[time=%s, type=%s, equiplet=%s]", time, type, equiplet);
		}
	}
}