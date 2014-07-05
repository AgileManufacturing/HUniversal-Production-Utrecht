package simulation.simulation;

class Event implements Comparable<Event> {

	private double time;
	private EventType type;
	private String product;
	private String equiplet;
	private String debug;

	/**
	 * Create event
	 * used for: DONE and PRODUCT event
	 * 
	 * @param time
	 * @param type
	 * @param steps
	 */
	public Event(double time, EventType type, String debug) {
		this.time = time;
		this.type = type;
		this.debug = debug;
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
	public Event(double time, EventType type, String product, String equiplet, String debug) {
		this.time = time;
		this.type = type;
		this.product = product;
		this.equiplet = equiplet;
		this.debug = debug;
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
	public Event(double time, EventType type, String equiplet, String debug) {
		this.time = time;
		this.type = type;
		this.equiplet = equiplet;
		this.debug = debug;
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

	public String getProduct() {
		return product;
	}

	public String getEquiplet() {
		return equiplet;
	}

	@Override
	public String toString() {
		if (type == EventType.PRODUCT || type == EventType.DONE) {
			return String.format("Evt[time=%.2f, type=%s, debug=%s]", time, type, debug);
		} else if (type == EventType.START) {
			return String.format("Evt[time=%.2f, type=%s, product=%s, equipet=%s, debug=%s]", time, type, product, equiplet, debug);
		} else {
			if (type == EventType.FINISHED) {
				// remove this if, not possible only for now
				return String.format("Evt[time=%.2f, type=%s, should be product=%s, equipet=%s, debug=%s]", time, type, product, equiplet, debug);
			}
			return String.format("Evt:[time=%.2f, type=%s, equiplet=%s, debug=%s]", time, type, equiplet, debug);
		}
	}
}