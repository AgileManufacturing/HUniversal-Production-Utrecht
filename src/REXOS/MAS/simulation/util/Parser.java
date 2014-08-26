package simulation.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import simulation.mas.equiplet.Capability;
import simulation.mas.product.ProductStep;
import simulation.mas.product.ProductionStep;

public class Parser extends ParserPrimitives {

	public static String parseEquipletConfiguration(Position position, List<Capability> capabilities) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("position", parsePosition(position));
		json.put("capabilities", parseCapabilties(capabilities));
		return json.toString();
	}

	public static Pair<Position, List<Capability>> parseEquipletConfiguration(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("position") && json.has("capabilities")) {
			Position position = parsePosition(json.getJSONObject("position"));
			List<Capability> capabilities = parseCapabilties(json.getJSONArray("capabilities"));
			return new Pair<Position, List<Capability>>(position, capabilities);
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

	public static Pair<LinkedList<ProductStep>, Position> parseProductConfiguration(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("productSteps") && json.has("position")) {
			LinkedList<ProductStep> productSteps = parseProductSteps(json.getJSONArray("productSteps"));
			Position position = parsePosition(json.getJSONObject("position"));
			return new Pair<LinkedList<ProductStep>, Position>(productSteps, position);
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

	public static String parseCanExecute(double time, double deadline, LinkedList<ProductStep> productSteps) throws JSONException {
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

	public static String parseScheduleRequest(ArrayList<ProductionStep> steps, double deadline) throws JSONException {
		JSONArray list = new JSONArray();
		for (ProductionStep productionStep : steps) {
			JSONObject json = new JSONObject();
			json.put("service", productionStep.getService());
			json.put("criteria", parseMap(productionStep.getCriteria()));
			json.put("time", productionStep.getStart());
			json.put("deadline", deadline);
			list.put(json);
		}
		return list.toString();
	}

	public static List<Tuple<Double, Double, String, Map<String, Object>>> parseScheduleRequest(String source) throws JSONException {
		List<Tuple<Double, Double, String, Map<String, Object>>> result = new ArrayList<>();
		JSONArray list = new JSONArray(source);
		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("service") && json.has("criteria") && json.has("time") && json.has("deadline")) {
				double time = json.getDouble("time");
				double deadline = json.getDouble("deadline");
				String service = json.getString("service");
				Map<String, Object> criteria = parseMap(json.getJSONArray("criteria"));

				result.add(new Tuple<Double, Double, String, Map<String, Object>>(time, deadline, service, criteria));
			} else {
				throw new JSONException("Parser: parsing scheduling failed to parse " + source);
			}
		}
		return result;
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

	public static Pair<Position, List<Pair<String, String>>> parseTravelRouteRequest(String source) throws JSONException {
		List<Pair<String, String>> routes = new ArrayList<Pair<String, String>>();
		JSONObject json = new JSONObject(source);
		if (json.has("position") && json.has("routes")) {
			Position position = parsePosition(json.getJSONObject("position"));
			JSONArray list = json.getJSONArray("routes");
			for (int i = 0; i < list.length(); i++) {
				JSONObject item = list.getJSONObject(i);
				if (item.has("from") && item.has("to")) {
					String from = item.getString("from");
					String to = item.getString("to");
					routes.add(new Pair<String, String>(from, to));
				} else {
					throw new JSONException("Parser: parsing travel time request failed to parse item " + source);
				}
			}
			return new Pair<Position, List<Pair<String, String>>>(position, routes);
		} else {
			throw new JSONException("Parser: parsing travel time request failed to parse " + source);
		}
	}

	public static String parseTravelRouteRequest(Position position, List<Pair<String, String>> routes) throws JSONException {
		JSONObject json = new JSONObject();
		JSONArray list = new JSONArray();
		for (Pair<String, String> route : routes) {
			JSONObject item = new JSONObject();
			item.put("from", route.first);
			item.put("to", route.second);
			list.put(item);
		}
		json.put("position", parsePosition(position));
		json.put("routes", list);
		return json.toString();
	}

	public static List<Pair<Position, Position>> parseTravelTimeRequest(String source) throws JSONException {
		List<Pair<Position, Position>> routes = new ArrayList<>();
		JSONArray list = new JSONArray(source);
		for (int i = 0; i < list.length(); i++) {
			JSONObject item = list.getJSONObject(i);
			if (item.has("from") && item.has("to")) {
				Position from = parsePosition(item.getJSONObject("from"));
				Position to = parsePosition(item.getJSONObject("to"));
				routes.add(new Pair<Position, Position>(from, to));
			} else {
				throw new JSONException("Parser: parsing travel time request failed to parse item " + source);
			}
		}
		return routes;

	}

	public static String parseTravelTimeRequest(Set<Pair<Position, Position>> routes) throws JSONException {
		JSONArray list = new JSONArray();
		for (Pair<Position, Position> route : routes) {
			JSONObject item = new JSONObject();
			item.put("from", parsePosition(route.first));
			item.put("to", parsePosition(route.second));
			list.put(item);
		}
		return list.toString();
	}

	public static Map<Pair<String, String>, Double> parseTravelRoutes(String source) throws JSONException {
		Map<Pair<String, String>, Double> travelTimes = new HashMap<Pair<String, String>, Double>();
		JSONArray list = new JSONArray(source);

		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("from") && json.has("to") && json.has("time")) {
				String from = json.getString("from");
				String to = json.getString("to");
				double time = json.getDouble("time");
				travelTimes.put(new Pair<String, String>(from, to), time);
			} else {
				throw new JSONException("Parser: parsing travel time routes failed to parse item " + source);
			}
		}
		return travelTimes;
	}

	public static String parseTravelRoutes(Map<Pair<String, String>, Double> travelTimes) throws JSONException {
		JSONArray list = new JSONArray();
		for (Entry<Pair<String, String>, Double> entry : travelTimes.entrySet()) {
			JSONObject json = new JSONObject();
			json.put("from", entry.getKey().first);
			json.put("to", entry.getKey().second);
			json.put("time", entry.getValue());
			list.put(json);
		}
		return list.toString();
	}

	public static Map<Pair<Position, Position>, Double> parseTravelTimes(String source) throws JSONException {
		Map<Pair<Position, Position>, Double> travelTimes = new HashMap<>();
		JSONArray list = new JSONArray(source);

		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("from") && json.has("to") && json.has("time")) {
				Position from = parsePosition(json.getJSONObject("from"));
				Position to = parsePosition(json.getJSONObject("to"));
				double time = json.getDouble("time");
				travelTimes.put(new Pair<Position, Position>(from, to), time);
			} else {
				throw new JSONException("Parser: parsing product travel times failed to parse item " + source);
			}
		}
		return travelTimes;
	}

	public static String parseTravelTimes(Map<Pair<Position, Position>, Double> travelTimes) throws JSONException {
		JSONArray list = new JSONArray();
		for (Entry<Pair<Position, Position>, Double> entry : travelTimes.entrySet()) {
			JSONObject json = new JSONObject();
			json.put("from", parsePosition(entry.getKey().first));
			json.put("to", parsePosition(entry.getKey().second));
			json.put("time", entry.getValue());
			list.put(json);
		}
		return list.toString();
	}

	public static double parseProductDelayed(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("start")) {
			return json.getDouble("start");
		} else {
			throw new JSONException("Parser: parsing product delayed failed " + source);
		}
	}

	public static String parseProductDelayed(double start) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("start", start);
		return json.toString();
	}
}
