package MAS.simulation.util;

public class Triple<U, V, W> {

	/**
	 * The first element of this <code>Tuple</code>
	 */
	public U first;

	/**
	 * The second element of this <code>Tuple</code>
	 */
	public V second;

	/**
	 * The third element of this <code>Tuple</code>
	 */
	public W third;

	/**
	 * Constructs a new <code>Tuple</code> with the given values.
	 * 
	 * @param first
	 *            the first element
	 * @param second
	 *            the second element
	 * @param third
	 *            the third element
	 */
	public Triple(U first, V second, W third) {
		this.first = first;
		this.second = second;
		this.third = third;
	}

	@Override
	public String toString() {
		return "<" + first + "," + second + "," + third + ">";
	}
}