/**
 * @file BlackboardReader.java
 * @brief
 * @date Created: 2013-03-20
 *
 * @author Arjen van Zanten
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 *
 **/

import nl.hu.client.BlackboardClient;
import nl.hu.client.ISubscriber;

import java.io.IOException;
import java.util.ArrayList;

public class BlackboardReader implements ISubscriber {
    private BlackboardClient client;

    public BlackboardReader(String host, String database, String collection, ArrayList<String> topics) {
        client = new BlackboardClient(host, this);
        try {
            client.setDatabase(database);
            client.setCollection(collection);

            for (String topic : topics) {
                client.subscribe(topic);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onMessage(String topic, Object message) {
        System.out.println("Topic: " + topic + " message: " + message);
    }

    public static void main(String[] args) {
        if (args.length > 3) {
            String host = args[0];
            String database = args[1];
            String collection = args[2];

            System.out.println("Mongo host:" + host + "\n"
                    + "Database:" + database + "\n"
                    + "Collection:" + collection);

            ArrayList<String> topics = new ArrayList<String>(args.length - 3);
            for (int i = 3; i < args.length; i++) {
                topics.add(args[i]);
                System.out.println("Topic " + (i - 2) + ": " + args[i]);
            }

            System.out.println("Starting. Press q and then enter to close.");
            BlackboardReader blackboardReader = new BlackboardReader(host, database, collection, topics);

            try {
                while (true) {
                    if (System.in.read() == 'q') {
                        System.exit(0);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

}
