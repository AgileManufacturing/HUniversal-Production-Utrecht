package agents.equiplet_agent;

import jade.core.AID;

public class ScheduleLock {
	
	private boolean scheduleLock;
	private AID lockGivenTo;
	
	public ScheduleLock(){
		scheduleLock = false;
		lockGivenTo = null;
	}
	
	public synchronized boolean acquireScheduleLock(AID productAgent){
		if (scheduleLock){
			return false;
		}
		else {
			lockGivenTo = productAgent;
			return true;
		}
	}
	
	public synchronized boolean releaseLock(AID productAgent){
		if (productAgent.equals(lockGivenTo)){
			scheduleLock = false;
			lockGivenTo = null;
			return true;
		}
		return false;
	}
	
	public synchronized AID getCurrentOwnerOfLock(){
		if (scheduleLock){
			return lockGivenTo;
		}
		return null;
	}
}
