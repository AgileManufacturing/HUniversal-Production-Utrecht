package equipletAgent;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class EquipletAgent extends Agent {
	
	private int _canPerformStepId;
	
	public int getCanPerformStepId(){
		return _canPerformStepId;
	}
	
	
	private class WaitMsgBehaviour extends CyclicBehaviour{

		public void action() {
			try{
				ACLMessage msg = receive(MessageTemplate.MatchOntology("CanPerformStep"));
				if (msg != null){
						ACLMessage message = new ACLMessage(ACLMessage.DISCONFIRM);
						message.addReceiver(msg.getSender());
						message.setOntology("CanPerformStep");
						send(message);
				} else {
					block();
				}
			}
			catch(Exception e){
			System.out.println("Error : " + e);
			}
		}
	}
	//Sets up the equiplet. The canperformStep id is the id of the step this equiplet can perform
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