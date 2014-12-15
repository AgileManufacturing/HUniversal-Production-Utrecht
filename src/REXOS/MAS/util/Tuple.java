package MAS.util;

public class Tuple<T, U, V, W> {

	/**
	 * The first element of this <code>Tuple</code>
	 */
	public T first;

	/**
	 * The second element of this <code>Tuple</code>
	 */
	public U second;

	/**
	 * The third element of this <code>Tuple</code>
	 */
	public V third;

	/**
	 * The foutrh element of this <code>Tuple</code>
	 */
	public W fourth;

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
	public Tuple(T first, U second, V third, W fourth) {
		this.first = first;
		this.second = second;
		this.third = third;
		this.fourth = fourth;
	}

	@Override
	public String toString() {
		return "<" + first + "," + second + "," + third + "," + fourth + ">";
	}
}