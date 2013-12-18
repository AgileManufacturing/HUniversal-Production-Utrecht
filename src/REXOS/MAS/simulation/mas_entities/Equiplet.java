package simulation.mas_entities;

import java.util.Arrays;
import java.util.Date;
import java.util.StringTokenizer;

import simulation.Updateable;
import simulation.data.Capability;

public class Equiplet implements Updateable{

	private Capability[] capabilities;
	
	
	
	public Equiplet(String capabilitesCSV, String InterruptionsCSV){
		
	}
	
	public Equiplet(Capability[] capabilities) {
		this.capabilities = capabilities; 
	}
	
	public void getFreeTimeSlots(){
		
	}
	
	public boolean canPerformStep(Capability capability){
		return Arrays.asList(capabilities).contains(capability);
	}
	
	public double getLoad(){
		return 0.0;
	}
	
	public long getFirstFreeTimeSlot(){
		return 0l;
	}
	
	public boolean isScheduleLocked(){
		return true;
	}
	
	public void schedule(){
		
	}
	
	private void parseInterruptions(){
		
	}

	@Override
	public void update(Date time) {
		// TODO Auto-generated method stub
		
	}
}
