package equipletAgent;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class EquipletAgent extends Agent {
	
	private int _canPerformStepId;
	
	public int getCanPerformStepId(){
		return _canPerformStepId;
	}
	
	
	private class WaitMsgBehaviour extends CyclicBehaviour{

		public void action() {
			try{
				ACLMessage msg = receive(); 
				if (msg != null) { 
					System.out.println(msg.getContent());
				} else {
					System.out.println("No msg received");
					block();
				}
			}
			catch(Exception e){
			System.out.println("Error : " + e);
			}
		}
	}

	  protected void setup() {
			try {
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
				addBehaviour(behaviour);
				   Object[] args = getArguments();
				   _canPerformStepId = (int) args[0];
				System.out.println("Started! my id is : " + _canPerformStepId);
			} catch (Exception e) {
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  } 
	}