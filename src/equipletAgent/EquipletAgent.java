package equipletAgent;

import java.util.Random;

import newDataClasses.*;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class EquipletAgent extends Agent {

	private int _canPerformStepId;
	private ProductionStep _step;

	public int getCanPerformStepId() {
		return _canPerformStepId;
	}

	private class WaitMsgBehaviour extends CyclicBehaviour {

		public void action() {
			try {
				ACLMessage msg = receive();
				if (msg != null) {

					String convid = msg.getConversationId();
					switch (msg.getOntology()) {
					case "CanPerformStep":
						_step = (ProductionStep) msg.getContentObject();
					//	System.out.println("Received query at " + myAgent.getAID() + " if I can perform step: " + _step.getId());
						ACLMessage message = new ACLMessage(
								ACLMessage.DISCONFIRM);
						message.setOntology("CanPerformStep");
						message.addReceiver(msg.getSender());
						// Debuggin. Set false to disconfirm the requested step
						if (getRandomBoolean()) {
							message.setPerformative(ACLMessage.CONFIRM);
						}
						
						message.setConversationId(convid);
						send(message);
						break;
					case "GetProductionDuration":
						_step = (ProductionStep) msg.getContentObject();
						//System.out.println("Received query at " + myAgent.getAID() + " how long it would take to perform: " + _step.getId());
						ACLMessage message2 = new ACLMessage(ACLMessage.INFORM);
						message2.setOntology("GetProductionDuration");
						long timeslots = 15;
						message2.setContentObject(timeslots);
						message2.addReceiver(msg.getSender());
						message2.setConversationId(convid);
						send(message2);
						break;
					}
				} else {
					block();
				}
			} catch (Exception e) {
				System.out.println("Error : " + e);
			}
		}
	}
	public boolean getRandomBoolean() {
	    Random random = new Random();
	    return random.nextBoolean();
	}

	private String writeParamsToString(ParameterList p) {
		String[] Groups = new String[3];
		Groups[0] = "Color";
		Groups[1] = "Shape";
		Groups[2] = "loc";

		String returnString = "Parameters are: \n";

		for (int i = 0; i < Groups.length; i++) {
			returnString += "Group : " + Groups[i] + " \n";
			ParameterGroup pg = p.GetParameterGroup(Groups[i]);
			Parameter[] pga = pg.getParameters();
			for (int j = 0; j < pga.length; j++) {
				returnString += "Parameter : " + pg.getParameters()[j].getKey()
						+ " value: "
						+ pg.getParameterValue(pg.getParameters()[j].getKey());
			}

		}

		return returnString;
	}

	// Sets up the equiplet. The canperformStep id is the id of the step this
	// equiplet can perform
	protected void setup() {
		try {
			WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
			addBehaviour(behaviour);
			Object[] args = getArguments();
			_canPerformStepId = (int) args[0];
		} catch (Exception e) {
			System.out.println("Exited with: " + e);
			doDelete();
		}
	}
}