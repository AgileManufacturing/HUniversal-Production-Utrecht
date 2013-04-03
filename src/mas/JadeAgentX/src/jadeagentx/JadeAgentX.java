/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jadeagentx;
import jadeagentx.EquipletAgent;
import jade.core.Agent;
import jade.wrapper.AgentController;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

/**
 *
 * @author wouter
 */
public class JadeAgentX extends Agent {

    /**
     * @param args the command line arguments
     */
    protected void setup(){
        try{
        System.out.println("starting a agent");
        
        Dictionary d=new Hashtable();
        d.put("x", "1");
        d.put("y", "2");
        
        ArrayList<String> inputParts = new ArrayList<String>();
        ArrayList<String> outputParts = new ArrayList<String>();
        
        inputParts.add("part1");
        inputParts.add("part2");
        
        outputParts.add("object1");
        outputParts.add("object2");

        Object[] ar = new Object[]{"1", d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent1", "jadeagentx.EquipletAgent", ar)).start();
        // TODO code application logic here
        
        ar = new Object[]{"2", d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent2", "jadeagentx.EquipletAgent", ar)).start();
        
        ar = new Object[]{"3", d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent3", "jadeagentx.EquipletAgent", ar)).start();
        
        ar = null;
        
        
        }catch(Exception e){
            
        }
    }
}
