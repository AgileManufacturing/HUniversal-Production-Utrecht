package MAS.equiplet;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONArray;
import org.json.JSONObject;

import jade.core.*; 
import jade.core.behaviours.*; 

import util.log.Logger;
import MAS.product.ProductionStep;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Tick;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

public class EQMessageAgentTester extends OneShotBehaviour {
	EQMessageAgent myTestAgent;
	
	public EQMessageAgentTester(EQMessageAgent myTestAgent){
		this.myTestAgent = myTestAgent;
	}

    public void action() {
        boolean testMastStateChange = true;
		boolean getModuleList = false;
		boolean getAllStateTest = false;
		boolean scheduleTest = false;
		boolean addRemoveModules = false;
		
		
		//Test a state change and the listener
		if(testMastStateChange){
			sleep(15000);
			//Register onChange listener
			myTestAgent.sendOnChangeRequest(EQMessageAgent.registerMastState);
			//Get current state
			myTestAgent.sendGetData(EQMessageAgent.getState);
			sleep(5000);
			//Change to save
			myTestAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			sleep(10000);//Give equiplet time to go through init state
			//Get state again
			myTestAgent.sendGetData(EQMessageAgent.getState);
			sleep(10000);
			//Return to offline
			myTestAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"OFFLINE\"}");
		}
		
		//Test get all modules
		if(getModuleList){
			myTestAgent.sendCommand(EQMessageAgent.insertJSON);
			sleep(5000);
			myTestAgent.sendGetData("{\"command\": \"GET_ALL_MODULES\"}");
			sleep(5000);
			myTestAgent.sendCommand(EQMessageAgent.deleteJSON);
			sleep(5000);
			myTestAgent.sendGetData("{\"command\": \"GET_ALL_MODULES\"}");
		}
		
		//Test get all possible states and modes
		if(getAllStateTest){
			myTestAgent.sendGetData(EQMessageAgent.getAllStates);
			sleep(5000);
			myTestAgent.sendGetData(EQMessageAgent.getAllModes);
			sleep(5000);
			myTestAgent.sendGetData(EQMessageAgent.getMode);
			sleep(5000);
		}
		
		//Test get schedule
		if(scheduleTest){
			
			try {
				myTestAgent.addScheduleJob(Parser.parseScheduleRequest(new ArrayList<ProductionStep>(), new Tick(50)));
			} catch (JSONException e) {
				e.printStackTrace();
			}	
			sleep(5000);
			myTestAgent.sendGetData(EQMessageAgent.getSchedule);
			sleep(5000);
		}

			
		
		//Add and remove modules
		if(addRemoveModules){
			myTestAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			myTestAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"NORMAL\"}");
			sleep(5000);
			myTestAgent.sendCommand(EQMessageAgent.insertJSON);
			//Should not work in normal mode
			sleep(5000);
			myTestAgent.sendCommand("{\"command\": \"CHANGE_EQUIPLET_MACHINE_STATE\", \"state\": \"SAFE\"}");
			sleep(5000);
			myTestAgent.sendCommand(EQMessageAgent.insertJSON);
			sleep(5000);
			myTestAgent.sendCommand(EQMessageAgent.deleteJSON);
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