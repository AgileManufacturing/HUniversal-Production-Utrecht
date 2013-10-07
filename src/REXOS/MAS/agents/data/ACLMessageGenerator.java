package agents.data;

import java.rmi.server.UID;

import jade.lang.acl.*;

public class ACLMessageGenerator {

	
	public static ACLMessage Generate(int performative){
		ACLMessage generated = new ACLMessage(performative);
		
		//add the testing variables
		generated.addUserDefinedParameter("message-id", new UID().toString());
		
		return generated;
	}
}
