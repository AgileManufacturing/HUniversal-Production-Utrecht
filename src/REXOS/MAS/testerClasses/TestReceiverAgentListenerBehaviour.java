package MAS.testerClasses;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;
import MAS.util.Ontology;

public class TestReceiverAgentListenerBehaviour extends Behaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private TestReceiverAgent testReceiver;
	private boolean done;

	public TestReceiverAgentListenerBehaviour(TestReceiverAgent testReceiver) {
		this.testReceiver = testReceiver;
		this.done = false;
	}

	@Override
	public void action() {
		// Listen only possible incoming conversation ids, note that otherwise the simulation would jam as the listener
		// receives messages that else where is waited upon
		// MessageTemplate template =
		// MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM),
		// MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.CONFIRM),
		// MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED))));
		MessageTemplate template = MessageTemplate.or(
				MessageTemplate.or(
						MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED), MessageTemplate.or(
								MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_RELEASE), MessageTemplate.or(
										MessageTemplate.MatchConversationId(Ontology.CONVERSATION_CAN_EXECUTE), MessageTemplate.or(
												MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE), 
													MessageTemplate.MatchConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND))))), 
				MessageTemplate.MatchConversationId(Ontology.CONVERSATION_INFORMATION_REQUEST)
		);
		
		ACLMessage msg = testReceiver.blockingReceive(template);
		if (msg != null) {
			System.out.printf("EA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", testReceiver.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());
			
			switch (msg.getPerformative()) {
			// messagetype holding the requested state for the equiplet
			case ACLMessage.PROPOSE:
				if(msg.getConversationId().equals(Ontology.CONVERSATION_EQUIPLET_COMMAND)){
					handleEquipletCommand(msg);
				}
				break;
			default:
				break;
			}
		}
	}

	private void handleEquipletCommand(ACLMessage msg) {
		if(msg != null){
			try{
				JSONObject command = new JSONObject(msg.getContent());
				
				//Debug output
				Logger.log("Content of ACL message: " + command.toString());
				
				//Identifying modules
				String requestedEquipletCommand = command.getString("requested-equiplet-command").toString();
				
				// Program if statements that will appropriately handle messages sent to the equiplet agent.
				if(requestedEquipletCommand.equals("TESTMESSAGE")){
					String agentName = command.getString("name");
					int messageID = command.getInt("id");
					long time = command.getLong("time");
					testReceiver.logMessage(agentName, messageID, time);
				}else{
					Logger.log("An error occured while deserializing the ACLMessage, missing info or command not recognized.");
				}
				
			//Error handling
			} catch (JSONException e) {
				Logger.log("Invalid JSON.");
			}
		}		
	}

	@Override
	public boolean done() {
		return done;
	}
}