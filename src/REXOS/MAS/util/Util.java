package MAS.util;

import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

public class Util {

	public static String formatArray(Map<?, ?> map) {
		StringBuffer buffer = new StringBuffer();
		for (Entry<?, ?> item : map.entrySet()) {
			buffer.append("\n" + item.getKey().toString() + " \t= ");
			buffer.append(item.getValue());
		}
		return buffer.toString();
	}

	public static String formatSet(Set<?> set) {
		StringBuffer buffer = new StringBuffer();
		for (Object obj : set) {
			buffer.append("\n\t\t" + obj.toString());
		}
		return buffer.toString();
	}

	public static <T, E> String formatPairList(TreeSet<Pair<T, E>> paths) {
		StringBuffer buffer = new StringBuffer();
		for (Pair<?, ?> item : paths) {
			buffer.append("\n" + item.first + " \t= ");
			buffer.append(item.second);
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

	public static TreeMap<Tick, Float> movingAverage(TreeMap<Tick, Float> map, int period) {
		// TreeMap<Tick, Double> newMap = new TreeMap<Tick, Double>();
		Queue<Float> window = new LinkedList<>();
		float sum = 0f;

		assert period > 0 : "Period must be a positive integer";

		for (Entry<Tick, Float> entry : map.entrySet()) {
			sum += entry.getValue();
			window.add(entry.getValue().floatValue());
			if (window.size() > period) {
				sum -= window.remove();
			}

			map.put(entry.getKey(), sum / window.size());
		}
		return map;
	}
}
