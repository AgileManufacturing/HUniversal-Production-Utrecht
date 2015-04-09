package MAS.testerClasses;


import MAS.util.Ontology;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

public class TestAgent extends Agent {

	private static final long serialVersionUID = 1L;
	int amountOfMessages = 0;
	
	protected void setup() {
		addBehaviour(new TestAgentListenerBehaviour(this));
		System.out.println("Agent: " + getLocalName() + " started");
	}
	
	public void sendMessage(){
		amountOfMessages++;
		// get agent.
		// TestReceiverAgent
		String name = "TRA";
		AID agent = new AID(name, AID.ISLOCALNAME);
		//send message.
		String JSONMessage = "{\n 'requested-equiplet-command': 'TESTMESSAGE',\n 'name': '" + getLocalName() + "', \n 'id':" + amountOfMessages +",\n 'time':" + System.currentTimeMillis() + "\n }";
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.addReceiver(agent);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
		message.setContent(JSONMessage);
		send(message);
		System.out.println("send");
	}


}
