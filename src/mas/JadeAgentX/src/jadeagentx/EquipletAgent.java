/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jadeagentx;
/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import java.util.*;

/**
 *
 * @author Thierry
 */
public class EquipletAgent extends Agent {

    private int timeSlotTime;
    private Dictionary parameters;
    private String productionType;
    private ArrayList inputParts, outputParts;
    private int productStep;

    @Override
    protected void setup() {
        // Printout a welcome message
        System.out.println("Hello! Special-agent " + getAID().getName() + " is waiting for a target to eliminate.");

        Object[] args = getArguments();
        if (args != null && args.length > 0) {
            productionType = (String) args[0];
            parameters = (Dictionary) args[1];
            inputParts = (ArrayList) args[2];
            outputParts = (ArrayList) args[3];



            addBehaviour(new TickerBehaviour(this, 5000) {
                @Override
                protected void onTick() {
                    //myAgent.addBehaviour(new RequestPerformer());
                    System.out.println(getAID().getName() + " checking messages");
                    ACLMessage msg = receive();
                    if (msg != null) {
                        //msg.setOntology("CanPerformStep");
                        // Process the message
                        System.out.println(getAID().getName() + " reporting: message received");

                        // deserialize content 
                        String messageID = msg.getConversationId();
                        String content = msg.getContent();
                        
                        String Ontology = msg.getOntology();
                        System.out.println(msg.getOntology());
                        ACLMessage confirmationMsg = new ACLMessage(ACLMessage.DISCONFIRM);
                        switch(Ontology){
                            case "CanPerformSteps": 
                                if(Integer.parseInt(content)== Integer.parseInt(productionType)){
                                    confirmationMsg.setPerformative(ACLMessage.CONFIRM);
                                    confirmationMsg.setContent("Dit is mogelijk");
                                    System.out.println("Dit is mogelijk");
                                }
                                else{
                                    confirmationMsg.setPerformative(ACLMessage.DISCONFIRM);
                                    confirmationMsg.setContent("Dit is niet mogelijk");
                                    System.out.println("Dit is niet mogelijk");
                                }
                            case "GetProductionDuration":
                                break;
                              
                        }
                        
                        System.out.println(productionType);
                        System.out.println(parameters);
                        System.out.println(inputParts);
                        System.out.println(outputParts);
                        
                        myAgent.send(confirmationMsg);
                        
                        
                        // na deserialization is het een list of dictionary,

                        //zoek uit hoe je een string terug serialized naar whatever it was before serialization

                        // haal het content object eruit, 

                        // de rest spreekt voorzich

                        confirmationMsg.createReply();
                        


                    }
                }
            });
        }

    }
}