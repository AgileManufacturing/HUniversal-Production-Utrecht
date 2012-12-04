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
import nl.hu.message.MessageBuilder;
import java.util.HashMap; 
import java.util.Scanner;
import nl.hu.client.ISubscriber;
import java.util.Date;

public class DummyAgent extends Agent implements ISubscriber
{
    private BlackboardClient client;
	private MessageBuilder builder = new MessageBuilder();
	private String database = "REXOS";
	private String topic = "instruction"; 
	private String collection = "blackboard";
	
	public void setup()
	{
		
		client = new BlackboardClient("localhost");
		
		try{
		client.setDatabase(database);	
		client.setCollection(collection);
		client.subscribe(topic);
		client.setCallback(this);
		}catch(Exception e){e.printStackTrace();}	

		this.addBehaviour(new CyclicBehaviour()
		{
			@Override
			public void action() 
			{
				try
				{
					while(true)
					{
						client.read(true);
					}
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}					

			}
		});

	}

	public void onMessage(String json)
	{
		client.removeFirst();
		System.out.print(" "+System.nanoTime() + "\n");
	//	System.out.println(json);
	}
}