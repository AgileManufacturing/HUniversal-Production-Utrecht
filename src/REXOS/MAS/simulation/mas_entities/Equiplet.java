package simulation.mas_entities;

import java.util.StringTokenizer;

public class Equiplet {

	private int[] capabilities;
	
	
	
	public Equiplet(String capabilitesCSV, String InterruptionsCSV){
		
	}
	
	public void getFreeTimeSlots(){
		
	}
	
	public void schedule(){
		
	}
	
	private void parseCapabilitiesCSV(String capabilitiesCSV){
		StringTokenizer stringTokenizer = new StringTokenizer(capabilitiesCSV, ",");
		capabilities = new int[stringTokenizer.countTokens()];
		int iCapabilities = 0;
		while (stringTokenizer.hasMoreTokens()){
			
		}
	}
	
	private void parseInterruptions(){
		
	}
}
