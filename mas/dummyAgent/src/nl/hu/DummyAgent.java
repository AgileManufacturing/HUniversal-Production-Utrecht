package nl.hu;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.TickerBehaviour;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;
import java.util.ArrayList;
import nl.hu.client.BlackboardClient;
import nl.hu.message.MessageBuilder;

public class DummyAgent extends Agent 
{

	public void setup()
	{
		BlackboardClient client = new BlackboardClient("localhost");	
		client.setDatabase("REXOS");	
		client.setCollection("blackboard");
		MessageBuilder builder = new MessageBuilder();
		builder.add("topic", "instruction");
		builder.add("message.destination", "DeltaRobotNode");
		builder.add("command", "moveToPoint");
		builder.add("message.payload.x", 5);
		builder.add("message.payload.y", 5);
		builder.add("message.payload.z", 5);
		try{
		client.insert(builder.buildMessage(MessageBuilder.MessageType.GET));
		}catch(Exception e){
		    e.printStackTrace();
		}
	}


}