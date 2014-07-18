package simulation.util;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Parser extends ParserPrimitives {

	public static String parseEquipletConfiguration(Position position, List<Capability> capabilities, Map<String, Double> productionTimes) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("position", parsePosition(position));
		json.put("capabilities", parseCapabilties(capabilities));
		json.put("productionTimes", parseProductionTimes(productionTimes));
		return json.toString();
	}

	public static Triple<Position, List<Capability>, Map<String, Double>> parseEquipletConfiguration(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("position") && json.has("capabilities") && json.has("productionTimes")) {
			Position position = parsePosition(json.getJSONObject("position"));
			List<Capability> capabilities = parseCapabilties(json.getJSONArray("capabilities"));
			Map<String, Double> productionTimes = parseProductionTimes(json.getJSONArray("productionTimes"));
			return new Triple<Position, List<Capability>, Map<String, Double>>(position, capabilities, productionTimes);
		} else {
			throw new JSONException("Parser: parsing equiplet configuration failed to parse " + json);
		}
	}

	public static String parseProductConfiguration(LinkedList<ProductStep> productSteps, Position startPosition) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("productSteps", parseProductSteps(productSteps));
		json.put("position", parsePosition(startPosition));
		return json.toString();
	}

	public static Pair<List<ProductStep>, Position> parseProductConfiguration(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("productSteps") && json.has("position")) {
			List<ProductStep> productSteps = parseProductSteps(json.getJSONArray("productSteps"));
			Position position = parsePosition(json.getJSONObject("position"));
			return new Pair<List<ProductStep>, Position>(productSteps, position);
		} else {
			throw new JSONException("Parser: parsing product agent configuration failed to parse " + source);
		}
	}

	@Deprecated
	public static String paresProductCreationAnswer(String equiplet, Position position) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("equiplet", equiplet);
		json.put("position", parsePosition(position));
		return json.toString();
	}

	@Deprecated
	public static Pair<String, Position> paresProductCreationAnswer(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("position") && json.has("equiplet")) {
			String equiplet = json.getString("equiplet");
			Position position = parsePosition(json.getJSONObject("position"));
			return new Pair<String, Position>(equiplet, position);
		} else {
			throw new JSONException("Parser: parsing product answer after creation failed to parse " + source);
		}
	}

	public static String parseCanExecute(double time, double deadline, List<ProductStep> productSteps) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("time", time);
		json.put("deadline", deadline);
		json.put("productSteps", parseProductSteps(productSteps));
		return json.toString();
	}

	public static Triple<Double, Double, List<ProductStep>> parseCanExecute(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("time") && json.has("deadline") && json.has("productSteps")) {
			double time = json.getDouble("time");
			double deadline = json.getDouble("deadline");
			List<ProductStep> productSteps = parseProductSteps(json.getJSONArray("productSteps"));

			return new Triple<Double, Double, List<ProductStep>>(time, deadline, productSteps);
		}
		throw new JSONException("Parser: parsing can execute failed to parse " + source);
	}

	/**
	 * The answer whether an equiplet can execute a list of product steps i.e. a service with criteria is constructed of three variables: The load of the equiplet, the position of
	 * the equiplet and the possibilities when a product step can be executed. The possibilities are a list for each service that can be executed, consisting of an index
	 * corresponding to the index of the product step, an estimate of executing the service, a list of times from and until it is possible to schedule the service
	 * 
	 * @param answer
	 *            List of services < index in production path, estimate production time, List of from and until time when possible>
	 * @param load
	 *            of equiplet
	 * @param position
	 *            of equiplet
	 * @return parsed json string
	 * @throws JSONException
	 *             when parsing failed
	 */
	public static String parseCanExecuteAnswer(List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer, Double load, Position position) throws JSONException {
		JSONObject json = new JSONObject();

		JSONArray list = new JSONArray();
		for (Triple<Integer, Double, List<Pair<Double, Double>>> triple : answer) {
			JSONObject service = new JSONObject();
			JSONArray possibilities = new JSONArray();
			for (Pair<Double, Double> option : triple.third) {
				JSONObject item = new JSONObject();
				item.put("from", option.first);
				item.put("until", option.second);
				possibilities.put(item);
			}
			service.put("index", triple.first);
			service.put("duration", triple.second);
			service.put("possibilities", possibilities);
			list.put(service);
		}

		json.put("answer", list);
		json.put("load", load);
		json.put("position", parsePosition(position));
		return json.toString();
	}

	public static Triple<List<Triple<Integer, Double, List<Pair<Double, Double>>>>, Double, Position> parseCanExecuteAnswer(String source) throws JSONException {
		JSONObject json = new JSONObject(source);

		if (json.has("answer") && json.has("load") && json.has("position")) {
			List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer = new ArrayList<Triple<Integer, Double, List<Pair<Double, Double>>>>();

			JSONArray list = json.getJSONArray("answer");
			for (int i = 0; i < list.length(); i++) {
				JSONObject service = list.getJSONObject(i);
				if (service.has("index") && service.has("duration") && service.has("possibilities")) {
					List<Pair<Double, Double>> possibilities = new ArrayList<Pair<Double, Double>>();

					JSONArray options = service.getJSONArray("possibilities");
					for (int j = 0; j < options.length(); j++) {
						JSONObject option = options.getJSONObject(j);
						if (option.has("from") && option.has("until")) {
							double from = option.getDouble("from");
							double until = option.getDouble("until");
							possibilities.add(new Pair<Double, Double>(from, until));
						} else {
							throw new JSONException("Parser: parsing can execute answer failed when parsing service " + option);
						}
					}

					answer.add(new Triple<Integer, Double, List<Pair<Double, Double>>>(service.getInt("index"), service.getDouble("duration"), possibilities));
				} else {
					throw new JSONException("Parser: parsing can execute answer failed when parsing service " + service);
				}
			}

			Double load = json.getDouble("load");
			Position position = parsePosition(json.getJSONObject("position"));
			return new Triple<>(answer, load, position);
		} else {
			throw new JSONException("Parser: parsing can execute answer failed to parse " + source);
		}
	}

	public static String parseSchedule(String service, Map<String, Object> criteria, double time, double deadline) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("service", service);
		json.put("criteria", parseMap(criteria));
		json.put("time", time);
		json.put("deadline", deadline);
		return json.toString();
	}

	public static Tuple<String, Map<String, Object>, Double, Double> parseSchedule(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("service") && json.has("criteria") && json.has("time") && json.has("deadline")) {
			String service = json.getString("service");
			Map<String, Object> criteria = parseMap(json.getJSONArray("criteria"));
			double time = json.getDouble("time");
			double deadline = json.getDouble("deadline");

			return new Tuple<String, Map<String, Object>, Double, Double>(service, criteria, time, deadline);
		} else {
			throw new JSONException("Parser: parsing scheduling failed to parse " + source);
		}
	}

	public static String parseConfirmation(boolean confirmation) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("confirmation", confirmation);
		return json.toString();
	}

	public static boolean parseConfirmation(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("confirmation")) {
			return json.getBoolean("confirmation");
		} else {
			throw new JSONException("Parser: parsing confirmation failed to parse " + source);
		}
	}

	public static String parseProductArrived(double time) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("time", time);
		return json.toString();
	}

	public static double parseProductArrived(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("time")) {
			return json.getDouble("time");
		} else {
			throw new JSONException("Parser: parsing product arrived failed to parse " + source);
		}
	}
}
