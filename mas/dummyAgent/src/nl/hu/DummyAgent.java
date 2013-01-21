/**
 * @file DummyAgent.java
 * @brief Provide a dummy agent that let the deltarobot move
 * @date Created: 2012-11-20
 *
 * @author Dick van der Steen
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

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

/**
 * DummyAgent that let the deltarobot move
 **/
public class DummyAgent extends Agent
{
    private BlackboardClient client;
	private String database = "REXOS";
	private String topic = "instruction";
	private String collection = "blackboard";
	private int i =0;

	public void setup()
	{
		// Create a new blackboard client
		client = new BlackboardClient("localhost");
		try{
			client.setDatabase(database);
			client.setCollection(collection);
			client.subscribe(topic);
		}catch(Exception e){
			e.printStackTrace();
		}

		// Add a new cyclic behaviour that moves the deltarobot 500 times
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
				try{
					System.out.print(System.nanoTime());
					client.insertJson(gson.toJson(mes));

					if(i == 500)
					{
						System.out.println("done!");
						blockingReceive();
					}
				}catch(Exception e){
					e.printStackTrace();
				}
			}
		});
	}
}
