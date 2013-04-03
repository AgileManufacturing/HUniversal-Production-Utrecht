package productAgent;

import java.util.ArrayList;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import dataClasses.equipletAgentData;

@SuppressWarnings("serial")
public class NegotiatorBehaviour extends CyclicBehaviour{

	private Productagent productAgent;
	private int currentStep = 0;
	private ArrayList<equipletAgentData> usableEquipletsList;
	
	public NegotiatorBehaviour(Productagent pa) {
		this.productAgent = pa;
	}
	
	@Override
	public void action() {
		// TODO Auto-generated method stub
		// Negotiate with the list containing possible equiplet agents. 
		// Store the plausible agents with the amount of time it will take (timeslots)
		
		//For testing purposes. Only 1 & 3 can perform his steps.
		productAgent.canPerfStepEquiplet = new ArrayList<equipletAgentData>();
		equipletAgentData d = new equipletAgentData(new AID("eqa1", AID.ISLOCALNAME));
		productAgent.canPerfStepEquiplet.add(d);
		
		d = new equipletAgentData(new AID("eqa1", AID.ISLOCALNAME));
		productAgent.canPerfStepEquiplet.add(d);
		//End for testing purposes
		
		usableEquipletsList = new ArrayList<equipletAgentData>();
		switch(currentStep){
			case 0: //Lets ask each EQ-A whether they can perform the step 
				startSending();
				currentStep = 1;
				break;
			case 1: //We have asked the EQ-A's. Now lets await their response.
				//If the response is positive, we might want to check the params aswell.
				startReceiving();
				break;			
			case 2: //We have asked the EQ-A's. Now lets await their response.
					//If the response is positive, we might want to check the params aswell.
				//startReceiving();
				break;
					
		}
	}
	
	//Starts receiving msgs from the equiplet agents.
	//It will only respond to CanPerformStep ontology msgs.
	public void startReceiving(){
		ACLMessage receive = productAgent.receive(MessageTemplate
				.MatchOntology("CanPerformStep"));
		if (receive != null) {
			try {
				AID senderId = receive.getSender();
				
				if(receive.getPerformative() == ACLMessage.CONFIRM){
					usableEquipletsList.add(new equipletAgentData(senderId));
				}
				
			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			block();
		}
	}

	//Starts sending msgs to the equiplet agents in the possible equiplet agent list.
	public void startSending(){
		//Foreach equipletagent
		for(equipletAgentData ead : productAgent.canPerfStepEquiplet){
			// Send the request to each agent. 
			// We want to know if it can perform the step with the given parameters.
			try {
				ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
				message.addReceiver(ead.AID);
				message.setOntology("CanPerformStep");
				message.setContent("1");
				productAgent.send(message);
				System.out.println("send message to: " + ead.AID);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

}
