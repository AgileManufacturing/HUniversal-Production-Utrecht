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
import com.google.gson.Gson;
import java.io.IOException;
 
import java.io.BufferedReader; 
import java.io.IOException; 
import java.io.InputStreamReader; 
import java.util.ArrayList; 
import nl.hu.client.BlackboardClient;
import nl.hu.message.MessageBuilder;
import java.util.HashMap; 

public class DummyAgent extends Agent 
{
    private BlackboardClient client = new BlackboardClient("localhost");	
	private MessageBuilder builder = new MessageBuilder();
	private String database = "REXOS";
	private String topic = "instruction"; 
	private String collection = "blackboard";

	public void setup()
	{
		
		builder.add("message.payload.maxAcceleration", 5);
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
				client.setDatabase(database);	
				client.setCollection(collection);
			
				Gson gson = new Gson();

				ArrayList<Point> points = new ArrayList<Point>();
				points.add(new Point(0,0,10, 50));
				points.add(new Point(0,0,-10 , 50));
				InstructionMessage a = new InstructionMessage("moveRelativePath", "DeltaRobotNode", "FIND_ID", null ,points);
				BlackboardMessage mes = new BlackboardMessage(topic,a);
				try
				{
					System.out.println(gson.toJson(mes));
					client.insertJson(gson.toJson(mes));
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}
					
								
				while(true){}
			}
		});

	}




}