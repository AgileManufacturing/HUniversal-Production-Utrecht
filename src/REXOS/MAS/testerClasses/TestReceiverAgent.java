package MAS.testerClasses;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import jade.core.Agent;

public class TestReceiverAgent extends Agent{

	private static final long serialVersionUID = 1L;
	private ArrayList<Message> messages;
	
	protected void setup() {
		messages = new ArrayList<Message>();
		addBehaviour(new TestReceiverAgentListenerBehaviour(this));
		System.out.println("Agent: " + getLocalName() + " started");
	}
	
	public void logMessage(String name, int messageID, long time){
		long timeNow = System.currentTimeMillis();
		System.out.println("Name: " + name + " ID: " + messageID + " time: " + time + " time now: " + timeNow);
		Message newMessage = new Message(name, messageID, time, timeNow);
		messages.add(newMessage);
		//500 agents and 5 messages. Yes this is ugly
		if(messages.size() == (500 * 5)){
			logToFile("ACLMessagesTest.csv");
		}
	}
	
	public void logToFile(String filename){
		try {
			FileWriter fw = new FileWriter(filename);
			BufferedWriter bf = new BufferedWriter(fw);
			bf.write("Name; ID; time send; time received; totalTime\n");
			for(int i = 0; i < messages.size(); i++){
				bf.write(messages.get(i).getName() + "; " + messages.get(i).getID() + "; " + messages.get(i).getTimeSend() + "; " + messages.get(i).getTimeReceived() + "; " + (messages.get(i).getTimeReceived()-messages.get(i).getTimeSend())+"\n");
			}
			bf.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
}
