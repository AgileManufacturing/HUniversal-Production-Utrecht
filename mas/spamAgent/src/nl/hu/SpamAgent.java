package nl.hu;
import java.util.Random;
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
import java.util.HashMap; 
import java.util.Scanner;
import nl.hu.client.ISubscriber;
import java.util.Date;
import java.io.*;
import java.util.concurrent.*; 

public class SpamAgent extends Agent
{
    private BlackboardClient client;
	private String database = "REXOS";
	private String topic = "instruction"; 
	private String collection = "blackboard";
	private int i =0;

	public void setup()
	{
		
		client = new BlackboardClient("localhost");
		
		try{
		client.setDatabase(database);	
		client.setCollection(collection);
		client.subscribe(topic);
		}catch(Exception e){e.printStackTrace();}	


		
		this.addBehaviour(new CyclicBehaviour()
		{
			@Override
			public void action() 
			{


				Gson gson = new Gson();
				
				ArrayList<Point> points = new ArrayList<Point>();
				points.add(new Point(i,i,i,i));
				
				i++;
				InstructionMessage a = new InstructionMessage("moveRelativePath", "DeltaRobotNode", "FIND_ID", null ,points);
				BlackboardMessage mes = new BlackboardMessage(topic,a);
				try
				{
	
					System.out.print(System.nanoTime());
					client.insertJson(gson.toJson(mes));
			
					
					if(i == 500)
					{
							System.out.println("done!");
						blockingReceive();
					}
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}	
				
					
			}
		});



	}
}
