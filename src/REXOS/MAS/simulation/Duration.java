package simulation;

public class Duration {
	static long parseDurationString(String input) {
		String[] fields = input.split(":");
		long output = 0;
		// hours
		output += Integer.parseInt(fields[0]) * 60 * 60 * 1000;
		// minutes
		output += Integer.parseInt(fields[1]) * 60 * 1000;
		// seconds
		output += Integer.parseInt(fields[2]) * 1000;
		return output;
	}
}
