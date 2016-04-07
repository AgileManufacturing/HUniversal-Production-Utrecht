package MAS.product;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;
import MAS.util.Ontology;
import SCADA.BasicAgentInfo;
import jade.core.AID;
import jade.core.behaviours.Behaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.lang.acl.ACLMessage;
import jade.proto.SubscriptionInitiator;

public class ProductAgent2ListenerBehaviour extends Behaviour {
    ProductAgent2 pa = null;
    
    private static final long serialVersionUID = 1L;
    
    public ProductAgent2ListenerBehaviour(ProductAgent2 pa) {
		this.pa = pa;
	}
    
    @Override
	public void action() {
        System.out.println("PA:" + pa.getLocalName() + " in action!");
        
		ACLMessage msg = pa.receive();
		if (msg != null) {
			switch (msg.getPerformative()) {
			
			case ACLMessage.REQUEST:
				//try {
					System.out.println("PA:" + pa.getLocalName() + " recieved new REQUEST command: " + msg.getContent().toString() );
				//} catch (JSONException e) {
					// TODO Auto-generated catch block
				//	e.printStackTrace();
				//}
				//break;
			default:
                System.out.println("PA:" + pa.getLocalName() + " recieved new unkown command: " + msg.getContent().toString() );
				break;
			}
		}else{
            block();
        }
	}
    /**
	 * done, default
	 * TODO implement method
	 * @return done 
	 */
	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return false;
	}
    
}