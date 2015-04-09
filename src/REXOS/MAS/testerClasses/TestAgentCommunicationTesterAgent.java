package MAS.testerClasses;

import MAS.util.Ontology;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

public class TestAgentCommunicationTesterAgent extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	protected void setup() {
		Object[] args = getArguments();
		if(args.length >= 5){
			String arg1 = args[0].toString();
			String agentBaseName = args[1].toString(); 
			int startingNumberAgentName = (int) args[2];
			int amountOfAgents = (int) args[3];
			int amountOfMessages = (int) args[4];
			if(arg1.equals("ACL")){
				if(!agentBaseName.equals("") && startingNumberAgentName >= 0 && amountOfAgents > 0 && amountOfMessages > 0){
					sendMessages("SENDMESSAGEACL", agentBaseName,startingNumberAgentName, amountOfAgents, amountOfMessages);
					System.out.println("Done ACL!");
				}
				System.err.println("Message settings are wrong ACL");
			
			}else if(arg1.equals("Socket")){
				if(startingNumberAgentName >= 0 && amountOfAgents > 0 && amountOfMessages > 0){
					sendMessages("SENDMESSAGESOCKET", agentBaseName, startingNumberAgentName, amountOfAgents, amountOfMessages);
					System.out.println("Done Socket!");
				}
				System.err.println("Message settings are wrong Socket");
			}
			else{
				System.err.println("Invalid argument agent: " + getLocalName() + "argument: " + arg1);
			}
		}
		else{
			System.err.println("Missing arguments agent: " + getLocalName());
		}
	}
	
	public void sendMessages( String messageType, String agentBaseName, int startingNumberAgentName, int amountOfAgents, int amountOfMessages){
		for(int i = 0; i < amountOfMessages; i++){
			for(int agentNumber = startingNumberAgentName; agentNumber < amountOfAgents + startingNumberAgentName; agentNumber++){
				// get agent.
				String name = agentBaseName + agentNumber;
				AID agent = new AID(name, AID.ISLOCALNAME);
				// send message.
				String JSONMessage = "{\n 'requested-equiplet-command': '" + messageType + "',\n 'no-content': 'true'\n }";
				ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
				message.addReceiver(agent);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
				message.setContent(JSONMessage);
				send(message);
			}
		}
	}
}
