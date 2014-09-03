package simulation.mas;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.json.JSONException;

import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;

public class TrafficManager extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Map<String, Position> equiplets;

	public TrafficManager(Map<String, Position> equiplets) {
		this.equiplets = equiplets;
	}

	@Override
	public void setup() {
		addBehaviour(new TraficListenerBehaviour());
	}

	protected Map<Pair<String, String>, Double> caculateTravelTimesEquiplets(List<Pair<String, String>> requests) {
		Map<Pair<String, String>, Double> travelTimes = new HashMap<Pair<String, String>, Double>();
		for (Pair<String, String> request : requests) {

			if (equiplets.containsKey(request.first) && equiplets.containsKey(request.second)) {
				double time = caclulateTravelTime(equiplets.get(request.first), equiplets.get(request.second));
				travelTimes.put(request, time);
			}
		}
		return travelTimes;
	}

	protected Map<Pair<Position, Position>, Double> caculateTravelTimesPosition(List<Pair<Position, Position>> requests) {
		Map<Pair<Position, Position>, Double> travelTimes = new HashMap<>();
		for (Pair<Position, Position> request : requests) {
			double time = caclulateTravelTime(request.first, request.second);
			travelTimes.put(request, time);
		}
		return travelTimes;
	}

	private double caclulateTravelTime(Position a, Position b) {
		int travelSquares = Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
		return 0.5 * travelSquares;
	}

	class TraficListenerBehaviour extends CyclicBehaviour {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			MessageTemplate template = MessageTemplate.MatchConversationId(Ontology.CONVERSATION_TRAVEL_TIME);
			ACLMessage msg = myAgent.blockingReceive(template);
			if (msg != null) {
				System.out.printf("TA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", myAgent.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());

				switch (msg.getPerformative()) {
				// query for information of the travel agent
				// TODO other way than match request route in position or equiplet names
				case ACLMessage.QUERY_IF:
					handleTravelRouteRequest(msg);
					break;
				case ACLMessage.QUERY_REF:
					handleTravelTimePositionRequest(msg);
					break;
				default:
					break;
				}
			}
		}

		private void handleTravelRouteRequest(ACLMessage message) {
			try {
				Pair<Position, List<Pair<String, String>>> request = Parser.parseTravelRouteRequest(message.getContent());
				Map<Pair<String, String>, Double> travelTimes = caculateTravelTimesEquiplets(request.second);

				// send travel time in reply
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.INFORM);
				reply.setContent(Parser.parseTravelRoutes(travelTimes));
				myAgent.send(reply);
			} catch (JSONException e) {
				e.printStackTrace();
			}
		}

		private void handleTravelTimePositionRequest(ACLMessage message) {
			try {
				List<Pair<Position, Position>> request = Parser.parseTravelTimeRequest(message.getContent());
				Map<Pair<Position, Position>, Double> travelTimes = caculateTravelTimesPosition(request);

				// send travel time in reply
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.INFORM);
				reply.setContent(Parser.parseTravelTimes(travelTimes));
				myAgent.send(reply);
			} catch (JSONException e) {
				e.printStackTrace();
			}
		}
	}

}
