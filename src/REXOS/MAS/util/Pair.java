package MAS.util;

import org.apache.commons.lang.builder.HashCodeBuilder;

public class Pair<U, V> {

	/**
	 * The first element of this <code>Pair</code>
	 */
	public U first;

	/**
	 * The second element of this <code>Pair</code>
	 */
	public V second;

	/**
	 * Constructs a new <code>Pair</code> with the given values.
	 * 
	 * @param first
	 *            the first element
	 * @param second
	 *            the second element
	 */
	public Pair(U first, V second) {
		this.first = first;
		this.second = second;
	}

	@Override
	public int hashCode() {
		// two randomly chosen prime numbers
		return new HashCodeBuilder(31, 71).append(first).append(second).toHashCode();
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
		Pair<?, ?> pair = (Pair<?, ?>) obj;
		return first.equals(pair.first) && second.equals(pair.second);
	}

	@Override
	public String toString() {
		return "<" + first + "," + second + ">";
	}
}
