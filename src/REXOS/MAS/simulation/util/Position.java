package simulation.util;

import org.apache.commons.lang.builder.EqualsBuilder;
import org.apache.commons.lang.builder.HashCodeBuilder;

public class Position {
	private int x;
	private int y;

	public Position(int x, int y) {
		this.x = x;
		this.y = y;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public String toString() {
		return String.format("(%d,%d)", x, y);
	}

	@Override
	public int hashCode() {
		// two randomly chosen prime numbers
		// if deriving: appendSuper(super.hashCode()).
		return new HashCodeBuilder(31, 71).append(x).append(y).toHashCode();
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
		Position rhs = (Position) obj;
		return new EqualsBuilder().append(x, rhs.getX()).append(y, rhs.getY()).isEquals();
	}
}
