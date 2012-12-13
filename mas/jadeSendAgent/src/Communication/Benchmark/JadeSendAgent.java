package Communication.Benchmark;

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
import java.util.Date;;
import java.io.*;
 
@SuppressWarnings("serial")
public class JadeSendAgent extends Agent {
  

    ArrayList<Long> times = new ArrayList<Long>();

    @Override
    protected void setup() {
        super.setup();
        addBehaviour(new CyclicBehaviour(this) {
        @Override
            public void action() 
            {
                ACLMessage temp = blockingReceive();   
		int x = Integer.parseInt(temp.getContent());
                for(int i=0; i < x; i++)
                {
                ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
                msg.addReceiver(new AID("receiver", AID.ISGUID));                
                msg.setContent("................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................");
                long sendingTime = System.nanoTime();
                send(msg);
                
                ACLMessage reply = blockingReceive();
                long receivingTime = System.nanoTime();
                times.add((receivingTime - sendingTime));   
                }
                try
		{	
		File yourFile = new File("out.txt");
		if(!yourFile.exists()) {
    			yourFile.createNewFile();
		} 
		FileWriter fstream = new FileWriter("out.txt");
  		BufferedWriter out = new BufferedWriter(fstream);
		for(Long l:times)
		{  		
		out.write(""+l+"\n");
		}	
		
  		//Close the output stream
  		out.close();
		System.out.println("done!");
  		}catch (Exception e){//Catch exception if any
  			System.err.println("Error: " + e.getMessage());
  		}  	
            }
        });
    }
}


