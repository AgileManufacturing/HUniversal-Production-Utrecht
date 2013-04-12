package hardwareAgent;

public class Module {
	//ik moet weten hoe lang iets duurt
	//stuur dit terug 
	long stepDuration;
	public Module(){
		
	}
	public void setStepDuration(long stepDuration){
		this.stepDuration = stepDuration;
	}
	public long getStepDuration(){
		return this.stepDuration;
		
	}
}
