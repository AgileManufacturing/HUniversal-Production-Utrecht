package simulation;

public class Duration {
	public static long parseDurationString(String input) throws NumberFormatException {
		String[] fields = input.split(":");
		
		if(fields.length != 3) {
			throw new NumberFormatException("Format must be hh:mm:ss. Hours, minutes and seconds");
		}
		
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
