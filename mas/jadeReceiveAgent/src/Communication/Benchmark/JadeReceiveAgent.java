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
import java.util.HashMap; 
import java.util.Scanner;
import nl.hu.client.ISubscriber;
import java.util.Date;

public class JadeReceiveAgent extends Agent
{
    
public void setup()
{
this.addBehaviour(new CyclicBehaviour()
{
@Override
public void action() 
{
ACLMessage message = blockingReceive();
ACLMessage reply = message.createReply();
reply.setContent(message.getContent());
send(reply);	

}
});

}

}
