package MAS.simulation.util;

public class Tick extends Number implements Comparable<Tick> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Class<?> type;
	private Long lTime;
	private Double dTime;

	public Tick() {
		this(System.currentTimeMillis());
	}

	public Tick(Long time) {
		type = Long.class;
		this.lTime = time;
		this.dTime = time.doubleValue();
	}

	public Tick(double time) {
		type = Double.class;
		this.dTime = time;
	}

	public <T> T get(Class<T> typeClass) {
		if (type.equals(typeClass)) {
			return typeClass.cast(dTime);
		} else if (type.equals(typeClass)) {
			return typeClass.cast(lTime);
		}
		return null;
	}

	@Override
	public String toString() {
		if (type.equals(Double.class)) {
			return String.format("%.2f", dTime);
		} else if (type.equals(Long.class)) {
			return lTime.toString();
		}
		// instead of illegal argument exception
		return dTime.toString();
	}

	@Override
	public double doubleValue() {
		return dTime;
	}

	@Override
	public long longValue() {
		return dTime.longValue();
	}

	@Override
	public int intValue() {
		return dTime.intValue();
	}

	@Override
	public float floatValue() {
		return dTime.floatValue();
	}

	@Override
	public int compareTo(Tick time) {
		return Double.compare(dTime, time.get(Double.class));
	}

	public boolean greaterThan(Tick time) {
		return dTime > time.get(Double.class);
	}

	public boolean greaterThan(int time) {
		return dTime > time;
	}

	public boolean greaterOrEqualThan(Tick time) {
		return dTime >= time.get(Double.class);
	}

	public boolean lessThan(Tick time) {
		return dTime < time.get(Double.class);
	}

	public boolean lessThan(double time) {
		return dTime < time;
	}

	public boolean lessOrEqualThan(Tick time) {
		return dTime <= time.get(Double.class);
	}

	public Tick times(int time) {
		return new Tick(dTime * time);
	}

	public Tick div(Tick time) {
		return new Tick(dTime / time.get(Double.class));
	}

	public Tick add(Tick time) {
		return new Tick(dTime + time.get(Double.class));
	}

	public Tick add(double time) {
		return new Tick(dTime + time);
	}

	public Tick minus(Tick time) {
		return new Tick(dTime - time.get(Double.class));
	}

	public Tick minus(double time) {
		return new Tick(dTime - time);
	}

	public Tick max(Tick time) {
		return new Tick(Math.max(dTime, time.get(Double.class)));
	}

	public Tick min(Tick time) {
		return new Tick(Math.min(dTime, time.get(Double.class)));
	}
}
