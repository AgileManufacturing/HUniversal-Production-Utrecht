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

public class SpamAgent extends Agent implements ISubscriber
{
    private BlackboardClient client;
	private MessageBuilder builder = new MessageBuilder();
	private String database = "REXOS";
	private String topic = "instruction"; 
	private String collection = "blackboard";
	private int i =0;
	public void setup()
	{
		
		client = new BlackboardClient("localhost",this);
		client.setDatabase(database);	
		client.setCollection(collection);
		try{
		client.subscribe(topic);
		}catch(Exception e){}	


		
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
				
					ACLMessage m = blockingReceive();
					System.out.println("sen:"+System.currentTimeMillis());
					client.insertJson(gson.toJson(mes));
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
		System.out.println(json);
	}


                public static int rand(int lo, int hi)
                {
                	Random randomGenerator = new Random();
                        int n = hi - lo + 1;
                        int i = randomGenerator.nextInt() % n;
                        if (i < 0)
                                i = -i;
                        return lo + i;
                }



}