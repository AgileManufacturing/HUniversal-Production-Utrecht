package MAS.simulation.util;

import java.util.Map;
import java.util.Map.Entry;

public class Util {

	public static String formatArray(Map<?, ?> map) {
		StringBuffer buffer = new StringBuffer();
		for (Entry<?, ?> item : map.entrySet()) {
			buffer.append("\n" + item.getKey().toString() + " \t= ");
			buffer.append(item.getValue());
		}
		return buffer.toString();
	}

	public static String formatMatrix(double[][] matrix) {
		StringBuffer buffer = new StringBuffer();
		for (int i = 0; i < matrix.length; i++) {
			for (int j = 0; j < matrix[i].length; j++) {
				buffer.append(String.format("%20s", matrix[i][j]));
			}
			buffer.append(System.getProperty("line.separator"));
		}
		return buffer.toString();
	}
}
