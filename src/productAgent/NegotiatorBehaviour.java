package productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.Serializable;
import java.util.Enumeration;
import java.util.Hashtable;

import newDataClasses.ProductionStep;

@SuppressWarnings("serial")
public class NegotiatorBehaviour extends CyclicBehaviour{

	private Productagent _productAgent;
	private int _currentStep;
	private Hashtable<ProductionStep, AID> _filteredEquipletsAndStepsList;
	
	public NegotiatorBehaviour(Productagent pa) {
		this._productAgent = pa;
		_currentStep = 0;
	}
	
	@Override
	public void action() {
		// Negotiate with the list containing possible equiplet agents. 
		// Store the plausible agents with the amount of time it will take (timeslots)
		
		//For testing purposes. Only 1 & 3 can perform his steps.
			_productAgent.canPerfStepEquiplet = new Hashtable<ProductionStep, AID>();
			_productAgent.canPerfStepEquiplet.put(_productAgent.productionStepList.get(0), new AID("eqa1", AID.ISLOCALNAME));
			_productAgent.canPerfStepEquiplet.put(_productAgent.productionStepList.get(1), new AID("eqa2", AID.ISLOCALNAME));
		//End hard coded production step list
		
		
		_filteredEquipletsAndStepsList = new Hashtable<ProductionStep, AID>();
		switch(_currentStep){
			case 0: 
				//Lets ask each EQ-A whether they can perform the step 
				startSending();
				_currentStep = 1;
				break;
			case 1: 
				//We have asked the EQ-A's. Now lets await their response.
				//If the response is positive, we might want to check the params aswell.
				startReceiving();
				break;
		}
	}
	
	//Starts receiving msgs from the equiplet agents.
	//It will only respond to CanPerformStep ontology msgs.
	public void startReceiving(){
		ACLMessage receive = _productAgent.receive(MessageTemplate
				.MatchOntology("CanPerformStep"));
		if (receive != null) {
			try {
				AID senderId = receive.getSender();
				
				ProductionStep step = (ProductionStep) receive.getContentObject();
				
				if(receive.getPerformative() == ACLMessage.CONFIRM){
					_filteredEquipletsAndStepsList.put(step, senderId);
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
		
		Enumeration keys = _productAgent.canPerfStepEquiplet.keys();
        while (keys.hasMoreElements()) {
        	//iterate over each element in the canPerfStepEquiplet hashtable
        	//retrieve the elements
        	ProductionStep step = (ProductionStep) keys.nextElement();
            AID Aid = _productAgent.canPerfStepEquiplet.get(step);
            
			try {
				
				ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
				message.addReceiver(Aid);
				message.setOntology("CanPerformStep");
				
				message.setContentObject(step.getParameterList()); // lets give em the parameters of this step.
				
				_productAgent.send(message);
				
				System.out.println("send message to: " + Aid);
				
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

}
