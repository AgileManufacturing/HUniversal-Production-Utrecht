package MAS.equiplet;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONArray;
import org.json.JSONObject;

import util.log.Logger;
import MAS.product.ProductionStep;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Tick;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

public class EQMessageBehaviour extends CyclicBehaviour{ 
	private static final long serialVersionUID = 1L;
			
	public void action (){ 
		ACLMessage msg = this.myAgent.receive();
		if(msg != null){
			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				try{
					JSONObject messageContent = new JSONObject(msg.getContent());
			
					//Identifying requested equiplet command
					String command = messageContent.getString("command");
					
					//Delegate command to corresponding functions
					switch(command){
					case "GET_CURRENT_EQUIPLET_STATE":
						Logger.log("CURRENT EQUIPLET STATE: " + messageContent.getString("state") + " !!!");
						break;
					default:
						Logger.log(msg.getSender().getLocalName() + " -> " + this.myAgent.getLocalName() + " - " + msg.getContent() + " - " + ACLMessage.getPerformative(msg.getPerformative()));
					}
					break;
				}catch(JSONException e){
					e.printStackTrace();
					Logger.log(msg.getSender().getLocalName() + " -> " + this.myAgent.getLocalName() + " - " + msg.getContent() + " - " + ACLMessage.getPerformative(msg.getPerformative()));
					break;
				}
			default:
				Logger.log(msg.getSender().getLocalName() + " -> " + this.myAgent.getLocalName() + " - " + msg.getContent() + " - " + ACLMessage.getPerformative(msg.getPerformative()));
			}
		}
	}
}