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
	
	private String insertJSON = "{\n\t\"requested-equiplet-command\": \"INSERT_MODULES\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public void setup(){
		sleep(1500);
		
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.setContent(insertJSON);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
		
		Logger.log("SEND MESSAGE");
		
		AID[] players = searchDF(Ontology.SERVICE_SEARCH_TYPE);
		for(int i = 0; i < players.length; i++){
			if(!players[i].equals(this.getAID())){
				message.addReceiver(players[i]);
				Logger.log("Add receiver: " + players[i].getLocalName());
			}
		}
		
		this.send(message);
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
