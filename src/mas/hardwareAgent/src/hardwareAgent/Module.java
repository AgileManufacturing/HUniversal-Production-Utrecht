package hardwareAgent;

/**
 * Authors: Thierry Gerritse 
 * Authors: Wouter Veen
 * Class: CheckForModules.java * 
 */

public class Module {

	private long _stepDuration;
	
	public Module(long stepDuration)throws Exception{
		
		if(stepDuration == 0l){
			throw new Exception("Duration can not be 0");
		}
		else{
		
			this._stepDuration = stepDuration;
			
		}
		
	}
	
	public void setStepDuration(long stepDuration){
		this._stepDuration = stepDuration;
	}
	
	public long getStepDuration(){
		return this._stepDuration;
		
	}
}
