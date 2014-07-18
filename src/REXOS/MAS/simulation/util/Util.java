package simulation.util;

import java.util.Map;
import java.util.Map.Entry;

public class Util {


	public static String formatArray(Map<?, ?> map) {
		StringBuffer buffer = new StringBuffer();
		for (Entry<?,?> item : map.entrySet()) {
			buffer.append("\n" + item.getKey().toString() + " \t= ");
			buffer.append(item.getValue());
		}
		return buffer.toString();
	}
}
