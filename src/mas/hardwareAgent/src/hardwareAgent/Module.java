package hardwareAgent;

import com.mongodb.BasicDBObject;

/**
 * Authors: Thierry Gerritse 
 * Authors: Wouter Veen
 * Class: CheckForModules.java * 
 */

public interface Module {
	
	public EquipletStepMessage[] getEquipletSteps(BasicDBObject parameters);
}
