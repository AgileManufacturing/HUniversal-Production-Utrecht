package MAS.util;

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

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.product.ProductionStep;

public class Parser extends ParserPrimitives {

	/**
	 * encode the equiplet configuration for starting an equiplet the configuration of an equiplet consists of the
	 * needed parameters to start the equiplet agent
	 * 
	 * counterpart {@link #parseEquipletConfiguration(String) parseEquipletConfiguration}
	 * 
	 * @param position
	 *            of the equiplet in the grid
	 * @param capabilities
	 *            of the equiplet
	 * @return the configuration in json format
	 * @throws JSONException
	 */
	public static String parseEquipletConfiguration(Position position, List<Capability> capabilities)
			throws JSONException {
		JSONObject json = new JSONObject();
		json.put("position", parsePosition(position));
		json.put("capabilities", parseCapabilties(capabilities));
		return json.toString();
	}

	/**
	 * decode the equiplet configuration for starting an equiplet the configuration of an equiplet consists of the
	 * needed parameters to start the equiplet agent
	 * 
	 * counterpart {@link #parseEquipletConfiguration(Position, List<Capability>) parseEquipletConfiguration}
	 * 
	 * @param source
	 *            encoded data in json format
	 * @return a tuple of the configuration of the equiplet
	 * @throws JSONException
	 */
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

	/**
	 * encode the product configuration for starting a product agent the product configurations are the needed
	 * parameters to start a product agent
	 * 
	 * counterpart {@link #parseProductConfiguration(String) parseProductConfiguration}
	 * 
	 * @param productSteps
	 *            of the product
	 * @param startPosition
	 *            of the product agent
	 * @return the configuration in json format
	 * @throws JSONException
	 */
	public static String parseProductConfiguration(LinkedList<ProductStep> productSteps, Position startPosition,
			Tick deadline) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("productSteps", parseProductSteps(productSteps));
		json.put("position", parsePosition(startPosition));
		json.put("deadline", parseTick(deadline));
		return json.toString();
	}

	/**
	 * decode the product configuration for starting a product agent the product configurations are the needed
	 * parameters to start a product agent
	 * 
	 * counterpart {@link #parseProductConfiguration(LinkedList<ProductStep>, Position) parseProductConfiguration}
	 * 
	 * @param source
	 *            encoded data in json format
	 * @return a tuple of the configuration of the product
	 * @throws JSONException
	 */
	public static Triple<LinkedList<ProductStep>, Position, Tick> parseProductConfiguration(String source)
			throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("productSteps") && json.has("position") && json.has("deadline")) {
			LinkedList<ProductStep> productSteps = parseProductSteps(json.getJSONArray("productSteps"));
			Position position = parsePosition(json.getJSONObject("position"));
			Tick deadline = parseTick(json.getJSONObject("deadline"));
			return new Triple<LinkedList<ProductStep>, Position, Tick>(productSteps, position, deadline);
		} else {
			throw new JSONException("Parser: parsing product agent configuration failed to parse " + source);
		}
	}

	/**
	 * encode the communication data to ask the equiplet agent if he can execute a list of product steps
	 * 
	 * counterpart {@link #parseCanExecute(String) parseCanExecute}
	 * 
	 * @param time
	 *            of the question/first product step
	 * @param deadline
	 *            till which is being search for available time
	 * @param productSteps
	 *            that are queried if the equiplet can execute
	 * @return json encoded data
	 * @throws JSONException
	 */
	public static String parseCanExecute(Tick time, Tick deadline, LinkedList<ProductStep> productSteps)
			throws JSONException {
		JSONObject json = new JSONObject();
		json.put("time", parseTick(time));
		json.put("deadline", parseTick(deadline));

		JSONArray steps = new JSONArray();
		for (ProductStep productStep : productSteps) {
			JSONObject step = new JSONObject();
			step.put("index", productStep.getIndex());
			step.put("service", productStep.getService());
			step.put("criteria", productStep.getCriteria());
			steps.put(step);
		}
		json.put("productSteps",steps);
		
		return json.toString();
	}

	/**
	 * the communication data to ask the equiplet agent if he can execute a list of product steps
	 * 
	 * counterpart {@link #parseCanExecute(Tick, Tick, LinkedList<ProductStep>) parseCanExecute}
	 * 
	 * @param source
	 *            encoded data in json format
	 * @return a tuple of the decoded data
	 * @throws JSONException
	 */
	public static Triple<Tick, Tick, List<Triple<Integer, String, JSONObject>>> parseCanExecute(String source)
			throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("time") && json.has("deadline") && json.has("productSteps")) {
			Tick time = parseTick(json.getJSONObject("time"));
			Tick deadline = parseTick(json.getJSONObject("deadline"));
			
			List<Triple<Integer, String, JSONObject>> productSteps = new ArrayList<>(); 
			JSONArray list =  json.getJSONArray("productSteps");
			for (int i = 0; i < list.length(); i++) {
				JSONObject item = list.getJSONObject(i);
				if (item.has("index") && item.has("service") && item.has("criteria")) {
					int index = item.getInt("index");
					String service = item.getString("service");
					JSONObject criteria = item.getJSONObject("criteria");
					productSteps.add(new Triple<>(index, service, criteria));
				} else {
					throw new JSONException("Parser: parsing product step failed to parse " + json);
				}
			}
			return new Triple<Tick, Tick, List<Triple<Integer, String, JSONObject>>>(time, deadline, productSteps);
		}
		throw new JSONException("Parser: parsing can execute failed to parse " + source);
	}

	/**
	 * The answer whether an equiplet can execute a list of product steps i.e. a service with criteria is constructed of
	 * three variables: The load of the equiplet, the position of the equiplet and the possibilities when a product step
	 * can be executed. The possibilities are a list for each service that can be executed, consisting of an index
	 * corresponding to the index of the product step, an estimate of executing the service, a list of times from and
	 * until it is possible to schedule the service
	 * 
	 * counterpart {@link #parseCanExecuteAnswer(String) parseCanExecuteAnswer}
	 * 
	 * @param answer
	 *            List of services < index in production path, estimate production time, List of from and until time
	 *            when possible>
	 * @param load
	 *            of equiplet
	 * @param position
	 *            of equiplet
	 * @return parsed json string
	 * @throws JSONException
	 *             when parsing failed
	 */
	public static String parseCanExecuteAnswer(List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> answer, Double load,
			Position position) throws JSONException {
		JSONObject json = new JSONObject();

		JSONArray list = new JSONArray();
		for (Triple<Integer, Tick, List<Pair<Tick, Tick>>> triple : answer) {
			JSONObject service = new JSONObject();
			JSONArray possibilities = new JSONArray();
			for (Pair<Tick, Tick> option : triple.third) {
				JSONObject item = new JSONObject();
				item.put("from", parseTick(option.first));
				item.put("until", parseTick(option.second));
				possibilities.put(item);
			}
			service.put("index", triple.first);
			service.put("duration", parseTick(triple.second));
			service.put("possibilities", possibilities);
			list.put(service);
		}

		json.put("answer", list);
		json.put("load", load);
		json.put("position", parsePosition(position));
		return json.toString();
	}

	/**
	 * decode the answer whether an equiplet can execute a list of product steps i.e. a service with criteria is
	 * constructed of three variables: The load of the equiplet, the position of the equiplet and the possibilities when
	 * a product step can be executed. The possibilities are a list for each service that can be executed, consisting of
	 * an index corresponding to the index of the product step, an estimate of executing the service, a list of times
	 * from and until it is possible to schedule the service
	 * 
	 * counterpart {@link #parseCanExecuteAnswer(List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>>, Double, Position)
	 * parseCanExecuteAnswer}
	 * 
	 * @param source
	 *            encoded data in json format
	 * @return a tuple of the decoded data
	 * @throws JSONException
	 */
	public static Triple<List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>>, Double, Position> parseCanExecuteAnswer(
			String source) throws JSONException {
		JSONObject json = new JSONObject(source);

		if (json.has("answer") && json.has("load") && json.has("position")) {
			List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> answer = new ArrayList<Triple<Integer, Tick, List<Pair<Tick, Tick>>>>();

			JSONArray list = json.getJSONArray("answer");
			for (int i = 0; i < list.length(); i++) {
				JSONObject service = list.getJSONObject(i);
				if (service.has("index") && service.has("duration") && service.has("possibilities")) {
					List<Pair<Tick, Tick>> possibilities = new ArrayList<Pair<Tick, Tick>>();

					JSONArray options = service.getJSONArray("possibilities");
					for (int j = 0; j < options.length(); j++) {
						JSONObject option = options.getJSONObject(j);
						if (option.has("from") && option.has("until")) {
							Tick from = parseTick(option.getJSONObject("from"));
							Tick until = parseTick(option.getJSONObject("until"));
							possibilities.add(new Pair<Tick, Tick>(from, until));
						} else {
							throw new JSONException("Parser: parsing can execute answer failed when parsing service "
									+ option);
						}
					}

					Tick duration = parseTick(service.getJSONObject("duration"));
					answer.add(new Triple<Integer, Tick, List<Pair<Tick, Tick>>>(service.getInt("index"), duration, possibilities));
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

	/**
	 * encode a request to schedule a list of product steps by an equiplet the data is list of [ product steps :: a
	 * tuple of < time, deadline, service, criteria > ]
	 * 
	 * counterpart {@link #parseScheduleRequest(String) parseScheduleRequest}
	 * 
	 * @param steps
	 *            is a list of production steps containing a service, criteria and start time
	 * @param deadline
	 *            of the product
	 * @return
	 * @throws JSONException
	 */
	public static String parseScheduleRequest(ArrayList<ProductionStep> steps, Tick deadline) throws JSONException {
		JSONArray list = new JSONArray();
		for (ProductionStep productionStep : steps) {
			JSONObject json = new JSONObject();
			json.put("index", productionStep.getIndex());
			json.put("service", productionStep.getService());
			json.put("criteria", productionStep.getCriteria());
			json.put("time", parseTick(productionStep.getStart()));
			json.put("deadline", parseTick(deadline));
			list.put(json);
		}
		return list.toString();
	}

	/**
	 * decode a request to schedule a list of product steps by an equiplet the data is list of [ product steps :: a
	 * tuple of < index, < time, deadline >, service, criteria > ]
	 * 
	 * counterpart {@link #parseScheduleRequest(ArrayList<ProductionStep>, Tick) parseScheduleRequest}
	 * 
	 * @param source
	 * @return a tuple of decoded schedule data
	 * @throws JSONException
	 */
	public static List<Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>> parseScheduleRequest(String source)
			throws JSONException {
		List<Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>> result = new ArrayList<>();
		JSONArray list = new JSONArray(source);
		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("index") && json.has("service") && json.has("criteria") && json.has("time")
					&& json.has("deadline")) {
				int index = json.getInt("index");
				Tick time = parseTick(json.getJSONObject("time"));
				Tick deadline = parseTick(json.getJSONObject("deadline"));
				String service = json.getString("service");
				JSONObject criteria = json.getJSONObject("criteria");

				result.add(new Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>(index, new Pair<Tick, Tick>(time, deadline), service, criteria));
			} else {
				throw new JSONException("Parser: parsing scheduling failed to parse " + source);
			}
		}
		return result;
	}

	/**
	 * parse confirmation this can be used for acknowledgement for receiving a message
	 * 
	 * counterpart {@link #parseConfirmation(String) parseConfirmation}
	 * 
	 * @param confirmation
	 * @return
	 * @throws JSONException
	 */
	public static String parseConfirmation(boolean confirmation) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("confirmation", confirmation);
		return json.toString();
	}

	/**
	 * parse confirmation this can be used for acknowledgement for receiving a message
	 * 
	 * counterpart {@link #parseConfirmation(boolean) parseConfirmation}
	 * 
	 * @param source
	 * @return
	 * @throws JSONException
	 */
	public static boolean parseConfirmation(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("confirmation")) {
			return json.getBoolean("confirmation");
		} else {
			throw new JSONException("Parser: parsing confirmation failed to parse " + source);
		}
	}

	/**
	 * encode the notification that a product is arrived by an equiplet
	 * 
	 * counterpart {@link #parseProductArrived(String) parseProductArrived}
	 * 
	 * @param time
	 *            of arrival
	 * @param index
	 *            of product step that is ready of product
	 * @return
	 * @throws JSONException
	 */
	public static String parseProductArrived(Tick time, int index) throws JSONException {
		return parseProductIndex(time, index);
		// JSONObject json = new JSONObject();
		// json.put("time", parseTick(time));
		// return json.toString();
	}

	/**
	 * decode the notification that a product is arrived by an equiplet
	 * 
	 * counterpart {@link #parseProductArrived(Tick, Integer) parseProductArrived}
	 * 
	 * @param source
	 * @return
	 * @throws JSONException
	 */
	public static Pair<Tick, Integer> parseProductArrived(String source) throws JSONException {
		return parseProductIndex(source);

		// JSONObject json = new JSONObject(source);
		// if (json.has("time")) {
		// return parseTick(json.getJSONObject("time"));
		// } else {
		// throw new JSONException("Parser: parsing product arrived failed to parse " + source);
		// }
	}

	public static Pair<Position, List<Pair<String, String>>> parseTravelRouteRequest(String source)
			throws JSONException {
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

	public static String parseTravelRouteRequest(Position position, List<Pair<String, String>> routes)
			throws JSONException {
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

	public static Map<Pair<String, String>, Tick> parseTravelRoutes(String source) throws JSONException {
		Map<Pair<String, String>, Tick> travelTimes = new HashMap<Pair<String, String>, Tick>();
		JSONArray list = new JSONArray(source);

		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("from") && json.has("to") && json.has("time")) {
				String from = json.getString("from");
				String to = json.getString("to");
				Tick time = parseTick(json.getJSONObject("time"));
				travelTimes.put(new Pair<String, String>(from, to), time);
			} else {
				throw new JSONException("Parser: parsing travel time routes failed to parse item " + source);
			}
		}
		return travelTimes;
	}

	public static String parseTravelRoutes(Map<Pair<String, String>, Tick> travelTimes) throws JSONException {
		JSONArray list = new JSONArray();
		for (Entry<Pair<String, String>, Tick> entry : travelTimes.entrySet()) {
			JSONObject json = new JSONObject();
			json.put("from", entry.getKey().first);
			json.put("to", entry.getKey().second);
			json.put("time", parseTick(entry.getValue()));
			list.put(json);
		}
		return list.toString();
	}

	public static Map<Pair<Position, Position>, Tick> parseTravelTimes(String source) throws JSONException {
		Map<Pair<Position, Position>, Tick> travelTimes = new HashMap<>();
		JSONArray list = new JSONArray(source);

		for (int i = 0; i < list.length(); i++) {
			JSONObject json = list.getJSONObject(i);
			if (json.has("from") && json.has("to") && json.has("time")) {
				Position from = parsePosition(json.getJSONObject("from"));
				Position to = parsePosition(json.getJSONObject("to"));
				Tick time = parseTick(json.getJSONObject("time"));
				travelTimes.put(new Pair<Position, Position>(from, to), time);
			} else {
				throw new JSONException("Parser: parsing product travel times failed to parse item " + source);
			}
		}
		return travelTimes;
	}

	public static String parseTravelTimes(Map<Pair<Position, Position>, Tick> travelTimes) throws JSONException {
		JSONArray list = new JSONArray();
		for (Entry<Pair<Position, Position>, Tick> entry : travelTimes.entrySet()) {
			JSONObject json = new JSONObject();
			json.put("from", parsePosition(entry.getKey().first));
			json.put("to", parsePosition(entry.getKey().second));
			json.put("time", parseTick(entry.getValue()));
			list.put(json);
		}
		return list.toString();
	}

	public static Pair<Tick, Integer> parseProductDelayed(String source) throws JSONException {
		return parseProductIndex(source);

		// JSONObject json = new JSONObject(source);
		// if (json.has("start")) {
		// return parseTick(json.getJSONObject("start"));
		// } else {
		// throw new JSONException("Parser: parsing product delayed failed " + source);
		// }
	}

	public static String parseProductDelayed(Tick start, int index) throws JSONException {
		return parseProductIndex(start, index);
		// JSONObject json = new JSONObject();
		// json.put("start", parseTick(start));
		// return json.toString();
	}

	public static String parseProductRelease(Tick time) throws JSONException {
		return parseTick(time).toString();
	}

	public static Tick parseProductRelease(String source) throws JSONException {
		return parseTick(new JSONObject(source));
	}

	/**
	 * parse product step index of a product this can be used for an equiplet informing a product that he start with
	 * processing of a product step or is finished with the product step
	 * 
	 * counterpart {@link #parseProductIndex(String) parseProductIndex}
	 * 
	 * @param time
	 *            of sending the information
	 * @param intdex
	 *            of product step
	 * @return the information encoded in json format
	 * @throws JSONException
	 */
	private static String parseProductIndex(Tick time, int index) throws JSONException {
		JSONObject json = new JSONObject();
		json.put("time", parseTick(time));
		json.put("index", index);
		return json.toString();
	}

	/**
	 * parse product step index of a product this can be used for an equiplet informing a product that he start with
	 * processing of a product step or is finished with the product step
	 * 
	 * counterpart {@link #parseProductIndex(Tick, Integer) parseProductIndex}
	 * 
	 * @param source
	 *            encoded data in json format
	 * @return a pair with the time and index
	 * @throws JSONException
	 */
	private static Pair<Tick, Integer> parseProductIndex(String source) throws JSONException {
		JSONObject json = new JSONObject(source);
		if (json.has("time") && json.has("index")) {
			Tick time = parseTick(json.getJSONObject("time"));
			int index = json.getInt("index");
			return new Pair<Tick, Integer>(time, index);
		} else {
			throw new JSONException("Parser: parsing product index failed " + source);
		}
	}

	public static String parseProductProcessing(Tick time, int index) throws JSONException {
		return parseProductIndex(time, index);
	}

	public static Pair<Tick, Integer> parseProductProcessing(String source) throws JSONException {
		return parseProductIndex(source);
	}

	public static String parseProductFinished(Tick time, int index) throws JSONException {
		return parseProductIndex(time, index);
	}

	public static Pair<Tick, Integer> parseProductFinished(String source) throws JSONException {
		return parseProductIndex(source);
	}
}
