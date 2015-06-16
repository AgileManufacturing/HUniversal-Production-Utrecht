package MAS.equiplet;

import util.log.Logger;
import MAS.util.Ontology;
import jade.core.AID;
import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

public class EQMessageAgent extends Agent {
	
	private String insertJSON = "{\n\t\"command\": \"INSERT_MODULE\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public void setup(){
		sleep(2000);
		
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.setContent(insertJSON);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
		
		//Logger.log("SEND MESSAGE");
		
		AID[] receivers = searchDF("EquipletAgent");
		for(int i = 0; i < receivers.length; i++){
			if(!receivers[i].equals(this.getAID())){
				message.addReceiver(receivers[i]);
				//Logger.log("Add receiver: " + receivers[i].getLocalName());
			}
		}
		
		this.send(message);
		Logger.log("Message was send");
	}
	
	public void takeDown(){
		
	}
	
	public AID [] searchDF(String service){
        DFAgentDescription dfd = new DFAgentDescription();
        ServiceDescription sd = new ServiceDescription();
        sd.setType( service );
        dfd.addServices(sd);
        
        SearchConstraints ALL = new SearchConstraints();
        ALL.setMaxResults(new Long(-1));
        try
        {
            DFAgentDescription[] result = DFService.search(this, dfd, ALL);
            AID[] agents = new AID[result.length];
            for (int i = 0; i<result.length; i++) 
                agents[i] = result[i].getName();
            return agents;
        }
        catch (FIPAException fe) { fe.printStackTrace(); }
        return null;
    }
	
	public void sleep(int ms){
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
