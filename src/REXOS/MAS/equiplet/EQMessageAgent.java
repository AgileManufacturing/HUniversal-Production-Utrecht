package MAS.equiplet;

import util.log.Logger;
import MAS.util.Ontology;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

/**
 * Tester class to send messages to the Equiplet Agent for testing responses
 * 
 * @author Kevin Bosman
 */
public class EQMessageAgent extends Agent {
	
	/**
	 * List of possible messages
	 * 
	 * @author Kevin Bosman
	 */
	private String insertJSON = "{\n\t\"command\": \"INSERT_MODULE\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	private String deleteJSON = "{\n\t\"command\": \"DELETE_MODULE\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	private String getAllStates = "{\"command\": \"GET_ALL_POSIBLE_STATES\"}";
	private String getAllModes = "{\"command\": \"GET_ALL_POSIBLE_MODES\"}";
	private String getState = "{\"command\": \"GET_CURRENT_EQUIPLET_STATE\"}";
	private String getSchedule = "{\"command\": \"GET_SCHEDULE\"}";
	private String getMode = "{\"command\": \"GET_CURRENT_MAST_MODE\"}";
	private String registerMastState = "{\"command\": \"ON_EQUIPLET_STATE_CHANGED\", \"action\": \"REGISTER_LISTENER\"}";
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	/**
	 * Send the message that will be tested
	 * 
	 * @author Kevin Bosman
	 */
	public void setup(){
		//Receive messages handling
		this.addBehaviour(new CyclicBehaviour (){ 
			private static final long serialVersionUID = 1L;
			public void action (){ 
				ACLMessage msg = this.myAgent.receive();
				if(msg != null){
					Logger.log(msg.getSender().getLocalName() + " -> " + this.myAgent.getLocalName() + " - " + msg.getContent() + " - " + ACLMessage.getPerformative(msg.getPerformative()));
				}
			}
		});
		
		//Use strings to avoid warnings...
		insertJSON.toString();
		deleteJSON.toString();
		getAllStates.toString();
		getAllModes.toString();
		getState.toString();
		getMode.toString();
		registerMastState.toString();
		getSchedule.toString();
		
		
		sleep(2000);
		
		//sendGetData(getSchedule);
		
		//Equiplet commands
		//sendCommand(deleteJSON);
		//sendCommand(insertJSON);
		
		//Get requests
		//sendGetData(getAllStates);
		//sendGetData(getAllModes);
		//sendGetData(getState);
		//sendGetData(getMode);
		
		//Listener test sequence
//		sendOnChangeRequest(registerMastState);
//		sleep(10000);
//		sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
//		sleep(10000);
//		sendGetData(getState);
//		sleep(10000);
//		sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"OFFLINE\"}");
	}
	
	public void takeDown(){
		
	}
	
	/**
	 * Send a equiplet command
	 * 
	 * @param String with JSON content
	 * @author Kevin Bosman
	 */
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
	
	/**
	 * Send a get data request
	 * 
	 * @param String with JSON content
	 * @author Kevin Bosman
	 */
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
	
	/**
	 * Send a on change message
	 * 
	 * @param String with JSON content
	 * @author Kevin Bosman
	 */
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
	
	/**
	 * Search the DF for a service. The messages will be send to all agents that are found
	 * 
	 * @param String of the service to search for
	 * @author Kevin Bosman
	 */
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
	
	/**
	 * Simple sleep with catch block
	 * 
	 * @param MS to sleep
	 * @author Kevin Bosman
	 */
	public void sleep(int ms){
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
