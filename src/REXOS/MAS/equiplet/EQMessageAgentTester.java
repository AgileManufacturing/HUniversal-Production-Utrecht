package MAS.equiplet;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONArray;
import org.json.JSONObject;

import util.log.Logger;
import MAS.product.ProductionStep;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Tick;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

public class EQMessageAgentTester implements Runnable {
	EQMessageAgent myAgent;
	
	public EQMessageAgentTester(EQMessageAgent myAgent){
		this.myAgent = myAgent;
	}

    public void run() {
        boolean testMastStateChange = true;
		boolean getModuleList = false;
		boolean getAllStateTest = false;
		boolean scheduleTest = false;
		boolean addRemoveModules = false;
		
		
		//Test a state change and the listener
		if(testMastStateChange){
			sleep(15000);
			//Register onChange listener
			myAgent.sendOnChangeRequest(EQMessageAgent.registerMastState);
			//Get current state
			myAgent.sendGetData(EQMessageAgent.getState);
			sleep(5000);
			//Change to save
			myAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			sleep(10000);//Give equiplet time to go through init state
			//Get state again
			myAgent.sendGetData(EQMessageAgent.getState);
			sleep(10000);
			//Return to offline
			myAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"OFFLINE\"}");
		}
		
		//Test get all modules
		if(getModuleList){
			myAgent.sendCommand(EQMessageAgent.insertJSON);
			sleep(5000);
			myAgent.sendGetData("{\"command\": \"GET_ALL_MODULES\"}");
			sleep(5000);
			myAgent.sendCommand(EQMessageAgent.deleteJSON);
			sleep(5000);
			myAgent.sendGetData("{\"command\": \"GET_ALL_MODULES\"}");
		}
		
		//Test get all possible states and modes
		if(getAllStateTest){
			myAgent.sendGetData(EQMessageAgent.getAllStates);
			sleep(5000);
			myAgent.sendGetData(EQMessageAgent.getAllModes);
			sleep(5000);
			myAgent.sendGetData(EQMessageAgent.getMode);
			sleep(5000);
		}
		
		//Test get schedule
		if(scheduleTest){
			
			try {
				myAgent.addScheduleJob(Parser.parseScheduleRequest(new ArrayList<ProductionStep>(), new Tick(50)));
			} catch (JSONException e) {
				e.printStackTrace();
			}	
			sleep(5000);
			myAgent.sendGetData(EQMessageAgent.getSchedule);
			sleep(5000);
		}

			
		
		//Add and remove modules
		if(addRemoveModules){
			myAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			myAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"NORMAL\"}");
			sleep(5000);
			myAgent.sendCommand(EQMessageAgent.insertJSON);
			//Should not work in normal mode
			sleep(5000);
			myAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			sleep(5000);
			myAgent.sendCommand(EQMessageAgent.insertJSON);
			sleep(5000);
			myAgent.sendCommand(EQMessageAgent.deleteJSON);
		}
    }
	
		/**
	 * Simple sleep with catch block
	 * 
	 * @param MS to sleep
	 * @author Kevin Bosman
	 */
	private void sleep(int ms){
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}