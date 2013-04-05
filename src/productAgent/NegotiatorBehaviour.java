package productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.Hashtable;

import newDataClasses.ProductionStep;

@SuppressWarnings("serial")
public class NegotiatorBehaviour extends CyclicBehaviour {

	private Productagent _productAgent;
	private int _currentStep;
	private Hashtable<ProductionStep, AID> _filteredEquipletsAndStepsList;

	public NegotiatorBehaviour(Productagent pa) {
		this._productAgent = pa;
		_currentStep = 0;
		_filteredEquipletsAndStepsList = new Hashtable<ProductionStep, AID>();
	}

	@Override
	public void action() {
		// Negotiate with the list containing possible equiplet agents.
		// Store the plausible agents with the amount of time it will take
		// (timeslots)
		/*
		 * switch (_currentStep) { case 0: // Lets ask each EQ-A whether they
		 * can perform the step startSending(); _currentStep = 1; break; case 1:
		 * // We have asked the EQ-A's. Now lets await their response.
		 * startReceiving(); break; }
		 */

		int timeSlots = -1;
		// foreachProductionstep in object[]{
		if (doConversationCanPerformStep(null, null))
			timeSlots = doConversationStepDuration(null, null);
		if (timeSlots > 0) // Everything went OK. Lets add this to the list
			timeSlots = doConversationStepDuration(null, null);
		// }

	}

	public boolean doConversationCanPerformStep(ProductionStep step, AID aid) {
		// Send the request with parameters
		// Check if it is a confirm.
		// if so, ask how long it will take
		// save yerr shit
		try {
			ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
			message.addReceiver(aid);
			message.setOntology("CanPerformStep");
			message.setContentObject(step);
			_productAgent.send(message);
			System.out.println("send message to: " + aid);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// So much for the sending. Lets try to receive a response.

		ACLMessage receive = _productAgent.receive(MessageTemplate
				.MatchOntology("CanPerformStep"));
		if (receive != null && receive.getSender() == aid) {
			try {
				if (receive.getPerformative() == ACLMessage.CONFIRM) {
					return true;
				}

			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			block();
		}
		return false;
	}

	public int doConversationStepDuration(ProductionStep step, AID aid) {
		// Send the request with parameters
		// Check if it is a confirm.
		// if so, ask how long it will take
		// save yerr shit
		try {
			ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
			message.addReceiver(aid);
			message.setOntology("CanPerformStep");
			message.setContentObject(step);
			_productAgent.send(message);
			System.out.println("send message to: " + aid);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// So much for the sending. Lets try to receive a response.

		ACLMessage receive = _productAgent.receive(MessageTemplate
				.MatchOntology("GetStepDuration"));
		if (receive != null && receive.getSender() == aid) {
			try {
				if (receive.getPerformative() == ACLMessage.CONFIRM) {
					return 1;
				}

			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			block();
		}
		return -1;
	}
	// Starts receiving msgs from the equiplet agents.
}