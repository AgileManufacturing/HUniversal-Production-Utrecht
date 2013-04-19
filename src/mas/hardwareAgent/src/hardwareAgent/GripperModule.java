package hardwareAgent;

import com.mongodb.BasicDBObject;

/**
 * Authors: Thierry Gerritse 
 * Class: GripperModule.java * 
 */

public class GripperModule implements Module{
	
	public GripperModule(){
	}
	
	@Override
	public EquipletStepMessage[] getEquipletSteps(BasicDBObject parameters) {
		// TODO Auto-generated method stub
		//return null;
		
		TimeData td = new TimeData(2l);
		
		EquipletStepMessage esm = new EquipletStepMessage(null, 1l, 2l, td);
		
		EquipletStepMessage[] dummyData = {esm};
		
		return dummyData;
		
	}	
	

}
