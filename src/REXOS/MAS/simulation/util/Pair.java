package simulation.util;

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
	public String toString() {
		return "<" + first + "," + second + ">";
	}
}
