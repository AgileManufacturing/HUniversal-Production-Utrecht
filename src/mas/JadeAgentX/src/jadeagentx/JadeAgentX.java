/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jadeagentx;
import equipletAgent.EquipletAgent;
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

        ArrayList<Long> capabilities1 = new ArrayList<Long>();
        capabilities1.add(3l);
        capabilities1.add(4l);
        
        
        Object[] ar = new Object[]{capabilities1, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent1", "equipletAgent.EquipletAgent", ar)).start();
        // TODO code application logic here
        ArrayList<Long> capabilities2 = new ArrayList<Long>();
        //capabilities2.clear();
        capabilities2.add(6l);
        capabilities2.add(3l);
        capabilities2.add(5l);
        
        ar = new Object[]{capabilities2, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent2", "equipletAgent.EquipletAgent", ar)).start();
        
        ArrayList<Long> capabilities3 = new ArrayList<Long>();
        //capabilities3.clear();
        capabilities3.add(1l);
        capabilities3.add(2l);
        capabilities3.add(4l);
        
        ar = new Object[]{capabilities3, d, inputParts, outputParts};
        ((AgentController)getContainerController().createNewAgent("equipletAgent3", "equipletAgent.EquipletAgent", ar)).start();
        
        ar = null;
        
        
        }catch(Exception e){
            
        }
    }
}
