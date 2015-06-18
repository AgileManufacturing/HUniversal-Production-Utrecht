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
	private String getAllStates = "{\"command\": \"GET_ALL_POSIBLE_STATES\"}";
	private String getAllModes = "{\"command\": \"GET_ALL_POSIBLE_MODES\"}";
	private String getState = "{\"command\": \"GET_CURRENT_EQUIPLET_STATE\"}";
	private String getMode = "{\"command\": \"GET_CURRENT_MAST_MODE\"}";
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public void setup(){
		sleep(2000);
		
		//Equiplet commands
		//sendCommand(insertJSON);
		
		//Get requests
		//sendGetData(getAllStates);
		//sendGetData(getAllModes);
		//sendGetData(getState);
		//sendGetData(getMode);
	}
	
	public void takeDown(){
		
	}
	
	public void sendCommand(String data){
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.setContent(data);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
		
		AID[] receivers = searchDF("EquipletAgent");
		for(int i = 0; i < receivers.length; i++){
			if(!receivers[i].equals(this.getAID())){
				message.addReceiver(receivers[i]);
				//Logger.log("Add receiver: " + receivers[i].getLocalName());
			}
		}
		
		this.send(message);
		Logger.log("(Command) Message was send: " + data);
	}
	
	public void sendGetData(String data){
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.QUERY_IF);
		message.setContent(data);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_GET_DATA);
		
		AID[] receivers = searchDF("EquipletAgent");
		for(int i = 0; i < receivers.length; i++){
			if(!receivers[i].equals(this.getAID())){
				message.addReceiver(receivers[i]);
				//Logger.log("Add receiver: " + receivers[i].getLocalName());
			}
		}
		
		this.send(message);
		Logger.log("(GetData) Message was send: " + data);
	}
	
	public void sendOnChangeRequest(String data){
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.setContent(data);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		
		AID[] receivers = searchDF("EquipletAgent");
		for(int i = 0; i < receivers.length; i++){
			if(!receivers[i].equals(this.getAID())){
				message.addReceiver(receivers[i]);
				//Logger.log("Add receiver: " + receivers[i].getLocalName());
			}
		}
		
		this.send(message);
		Logger.log("(OnChange) Message was send: " + data);
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
