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

import com.google.gson.Gson;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import nl.hu.client.BlackboardClient;
import nl.hu.client.ISubscriber;

import java.util.ArrayList;

/**
 * DummyAgent that makes the deltarobot move
 **/
public class DummyAgent extends Agent implements ISubscriber {
    private BlackboardClient client;
    private String database = "REXOS";
    private String topic = "instruction";
    private String collection = "blackboard";
    private int i = 0;

    public void setup() {
        // Create a new blackboard client
        client = new BlackboardClient("localhost", this);
        try {
            client.setDatabase(database);
            client.setCollection(collection);
            client.subscribe(topic);
        } catch (Exception e) {
            e.printStackTrace();
        }

	    System.out.println("Hello, I'm a dummy agent!");

        // Add a new cyclic behaviour that moves the deltarobot 500 times
        this.addBehaviour(new CyclicBehaviour() {
            @Override
            public void action() {
                Gson gson = new Gson();

                ArrayList<Point> points = new ArrayList<Point>();
                points.add(new Point(i, i, i, i));

                i++;
                InstructionMessage a = new InstructionMessage("moveRelativePath", "DeltaRobotNode", "FIND_ID", null, points);
                BlackboardMessage mes = new BlackboardMessage(topic, a);
                try {
                    //System.out.print(System.nanoTime());
                    Thread.sleep(500);
                    client.insertJson(gson.toJson(mes));

                    if (i == 500) {
                        System.out.println("done!");
	                    i = 0;
                        blockingReceive();
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    @Override
    public void onMessage(String topic, Object message) {
        //System.out.println("Topic: " + topic + " message: " + message);
    }
}
