package MAS.util;

import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import MAS.product.Node;

public class Util {

	public static String formatArray(Map<?, ?> map) {
		StringBuffer buffer = new StringBuffer();
		for (Entry<?, ?> item : map.entrySet()) {
			buffer.append("\n" + item.getKey().toString() + " \t= ");
			buffer.append(item.getValue());
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

	public static TreeMap<Tick, Double> movingAverage(TreeMap<Tick, Double> map, int period) {
		TreeMap<Tick, Double> newMap = new TreeMap<Tick, Double>();
		Queue<Double> window = new LinkedList<Double>();
		double sum = 0.0;

		assert period > 0 : "Period must be a positive integer";

		for (Entry<Tick, Double> entry : map.entrySet()) {
			sum += entry.getValue();
			window.add(entry.getValue());
			if (window.size() > period) {
				sum -= window.remove();
			}

			newMap.put(entry.getKey(), sum / window.size());
		}
		return newMap;
	}
}
