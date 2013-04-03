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

        ArrayList<Long> capabilities = new ArrayList<Long>();
        capabilities.add(3l);
        capabilities.add(4l);
        
        
        Object[] ar = new Object[]{capabilities, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent1", "equipletAgent.EquipletAgent", ar)).start();
        // TODO code application logic here
        
        capabilities.clear();
        capabilities.add(6l);
        capabilities.add(3l);
        capabilities.add(5l);
        
        ar = new Object[]{capabilities, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent2", "equipletAgent.EquipletAgent", ar)).start();
        
        capabilities.clear();
        capabilities.add(1l);
        capabilities.add(2l);
        capabilities.add(4l);
        
        ar = new Object[]{capabilities, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent3", "equipletAgent.EquipletAgent", ar)).start();
        
        ar = null;
        
        
        }catch(Exception e){
            
        }
    }
}
