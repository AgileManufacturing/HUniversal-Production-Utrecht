package simulation.util;

public class Tick {

	private long time;
	
	public Tick(long start) {
		this.time = start;
	}

	@Override
	public String toString() {
		return String.valueOf(time);
	}
}
