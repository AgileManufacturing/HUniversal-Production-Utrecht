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
    private BlackboardClient client = new BlackboardClient("localhost");	
	private MessageBuilder builder = new MessageBuilder();
	private String database = "REXOS";
	private String topic = "instruction"; 
	private String collection = "blackboard";

	public void setup()
	{
		
		try{
		//client.insert(builder.buildMessage(MessageBuilder.MessageType.GET));
		}catch(Exception e){
		    e.printStackTrace();
		}
		System.out.println("Send a message to this agent to move the DeltaRobotNode to relative points using content:");
		System.out.println("x@y@z");
		this.addBehaviour(new CyclicBehaviour()
		{
			@Override
			public void action() 
			{
				ACLMessage message = myAgent.blockingReceive();
				String content = message.getContent();
				System.out.println(content);
				String [] split = content.split("@");

				if(split.length == 3)
				{
					try
					{
						client.setDatabase(database);	
						client.setCollection(collection);
						builder.flush();
						builder.add("topic", topic);
						builder.add("message.command", "moveToRelativePoint");
						builder.add("message.destination","deltaRobotNode");				
						builder.add("message.payload.x", Integer.parseInt(split[0]));
						builder.add("message.payload.y", Integer.parseInt(split[1]));
						builder.add("message.payload.z", Integer.parseInt(split[2]));
						System.out.println(builder.buildMessage(MessageBuilder.MessageType.GET));
					
						client.insert(builder.buildMessage(MessageBuilder.MessageType.GET));
					}
					catch(Exception e){
						System.out.println("Wrong number filled in for x,y or z value");
					}
				}
				else
				{
					System.out.println("wrong string has been sent! please use the following syntax:");
					System.out.println("x@y@z");
				}
			}
		});

	}




}