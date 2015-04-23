package MAS.util;

import org.apache.commons.lang.builder.HashCodeBuilder;

public class Tick extends Number implements Comparable<Tick> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private double dTime;

	public Tick() {
		this(System.currentTimeMillis());
	}

	//
	//	public Tick(Long time) {
	//		type = Long.class;
	//		this.lTime = time;
	//		this.dTime = time.doubleValue();
	//	}

	public Tick(double time) {
		//		type = Double.class;
		this.dTime = time;
	}

	//	public <T> T get(Class<T> typeClass) {
	//		if (type.equals(typeClass)) {
	//			return typeClass.cast(dTime);
	//		} else if (type.equals(typeClass)) {
	//			return typeClass.cast(lTime);
	//		}
	//		return null;
	//	}

	@Override
	public String toString() {
		// 		if (type.equals(Double.class)) {
		return String.format("%.2f", dTime);
		//		} else if (type.equals(Long.class)) {
		//			return lTime.toString();
		//		}
		//		// instead of illegal argument exception
		//		return dTime.toString();
	}

	@Override
	public int hashCode() {
		return new HashCodeBuilder(29, 73).append(dTime).toHashCode(); // .append(lTime).toHashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null) {
			return false;
		}
		if (obj == this) {
			return true;
		}
		if (obj.getClass() != getClass()) {
			return false;
		}
		Tick tick = (Tick) obj;
		return dTime == tick.doubleValue();
	}

	@Override
	public double doubleValue() {
		return dTime;
	}

	@Override
	public long longValue() {
		return new Double(dTime).longValue();
	}

	@Override
	public float floatValue() {
		return new Double(dTime).floatValue();
	}

	@Override
	public int intValue() {
		return new Double(dTime).intValue();
	}

	@Override
	public int compareTo(Tick time) {
		return Double.compare(dTime, time.doubleValue());
	}

	public boolean greaterThan(Tick time) {
		return dTime > time.doubleValue();
	}

	public boolean greaterThan(int time) {
		return dTime > time;
	}

	public boolean greaterOrEqualThan(Tick time) {
		return dTime >= time.doubleValue();
	}

	public boolean lessThan(Tick time) {
		return dTime < time.doubleValue();
	}

	public boolean lessThan(double time) {
		return dTime < time;
	}

	public boolean lessOrEqualThan(Tick time) {
		return dTime <= time.doubleValue();
	}

	public Tick multiply(int time) {
		return new Tick(dTime * time);
	}

	public Tick multiply(double time) {
		return new Tick(dTime * time);
	}

	public Tick div(Tick time) {
		return new Tick(dTime / time.doubleValue());
	}

	public Tick add(Tick time) {
		return new Tick(dTime + time.doubleValue());
	}

	public Tick add(double time) {
		return new Tick(dTime + time);
	}

	public Tick minus(Tick time) {
		return new Tick(dTime - time.doubleValue());
	}

	public Tick minus(double time) {
		return new Tick(dTime - time);
	}

	public Tick max(Tick time) {
		return new Tick(Math.max(dTime, time.doubleValue()));
	}

	public Tick max(double time) {
		return new Tick(Math.max(dTime, time));
	}

	public Tick min(Tick time) {
		return new Tick(Math.min(dTime, time.doubleValue()));
	}
}
