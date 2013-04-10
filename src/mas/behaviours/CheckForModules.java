package behaviours;

/**
 * Author: Thierry Gerritse
 * Class: CheckForModules.java * 
 */

import hardwareAgent.HardwareAgent;

import java.io.IOException;

import newDataClasses.ProductionStep;

import org.bson.types.ObjectId;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import equipletAgent.ProductStepMessage;
import equipletAgent.ProductStepStatusCode;
import equipletAgent.EquipletAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

public class CheckForModules extends ReceiveBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
    private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("CheckForModules");
    private HardwareAgent hardwareAgent;
	
	/**
	 * Instantiates a new can check for module.
	 *
	 */
	public CheckForModules(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent)a;
	}
	
	@Override
	public void handle(ACLMessage message){
		Object contentObject = null;
		String contentString = message.getContent();

		try {
			contentObject = message.getContentObject();
		} catch (UnreadableException e) {
			//System.out.println("Exception Caught, No Content Object Given");
		}
		System.out.format("%s received message from %s (%s:%s)%n",
				myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(), contentObject == null ? contentString : contentObject);
		
		try{
			
			//do something
		
		} catch (Exception e) {
			e.printStackTrace();
			// TODO: ERROR HANDLING
			myAgent.doDelete();
		}
	}
}
