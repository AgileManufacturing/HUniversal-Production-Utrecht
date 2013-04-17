package hardwareAgent;

import com.mongodb.BasicDBObject;

/**
 * Authors: Thierry Gerritse 
 * Class: GripperModule.java * 
 */

public class GripperModule implements Module{
	
	private long _stepDuration = 10l;
	
	public GripperModule(){
		
		HardwareAgent hardwareAgent = new HardwareAgent();		
		hardwareAgent.RegisterModule("gripper", this);
	}
	
	@Override
	public EquipletStepMessage[] getEquipletSteps(BasicDBObject parameters) {
		// TODO Auto-generated method stub
		return null;
	}	
	

}
