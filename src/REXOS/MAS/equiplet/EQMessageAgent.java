package MAS.equiplet;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONArray;
import org.json.JSONObject;

import jade.core.*; 
import jade.core.behaviours.*; 

import util.log.Logger;
import MAS.product.ProductionStep;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Tick;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

/**
 * Tester class to send messages to the Equiplet Agent for testing responses and message handling
 * 
 * @author Kevin Bosman
 * @author Thomas Kok
 */
public class EQMessageAgent extends Agent {
	
	/**
	 * List of possible messages
	 * 
	 * @author Kevin Bosman
	 */
	public static final String insertJSON = "{\n\t\"command\": \"INSERT_MODULE\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	public static final String deleteJSON = "{\n\t\"command\": \"DELETE_MODULE\",\n\t\"modules\": [{\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"delta_robot_type_B\",\n\t\t\"serialNumber\": \"1\"\n\t}, {\n\t\t\"manufacturer\": \"HU\",\n\t\t\"typeNumber\": \"gripper_type_A\",\n\t\t\"serialNumber\": \"1\"\n\t}, ]\n}";
	public static final String getAllStates = "{\"command\": \"GET_ALL_POSIBLE_STATES\"}";
	public static final String getAllModes = "{\"command\": \"GET_ALL_POSIBLE_MODES\"}";
	public static final String getState = "{\"command\": \"GET_CURRENT_EQUIPLET_STATE\"}";
	public static final String getSchedule = "{\"command\": \"GET_SCHEDULE\"}";
	public static final String getMode = "{\"command\": \"GET_CURRENT_MAST_MODE\"}";
	public static final String registerMastState = "{\"command\": \"ON_EQUIPLET_STATE_CHANGED\", \"action\": \"REGISTER_LISTENER\"}";
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
		Logger.log("SETUP EQ MESSAGE AGENT START");
		
		ThreadedBehaviourFactory tbf = new ThreadedBehaviourFactory();
		
		//Receive messages handling
		this.addBehaviour(new EQMessageBehaviour());
		
		//Behaviour b = new EQMessageAgentTester(this);
		//addBehaviour(tbf.wrap(b));
		
		Logger.log("SETUP EQ MESSAGE AGENT DONE");
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
			}
		}
		
		this.send(message);
		Logger.log("Equiplet setter test message: " + data);
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
			}
		}
		
		this.send(message);
		Logger.log("Equiplet getter test message: " + data);
	}
	
	/**
	 * Send an onchange message
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
			}
		}
		
		this.send(message);
		Logger.log("Equiplet listner test message: " + data);
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
        return new AID[0];
    }
	
	/**
	 * Send a get data request
	 * 
	 * @param String with JSON content
	 * @author Kevin Bosman
	 */
	public void addScheduleJob(String data){
		//Send message
		ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
		message.setContent(data);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
		message.setReplyWith(Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis());
		
		AID[] receivers = searchDF("EquipletAgent");
		for(int i = 0; i < receivers.length; i++){
			if(!receivers[i].equals(this.getAID())){
				message.addReceiver(receivers[i]);
			}
		}
		
		this.send(message);
		Logger.log("Equiplet getter test message: " + data);
	}

}
