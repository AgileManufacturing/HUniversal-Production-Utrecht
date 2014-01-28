package agents.data_classes;

import jade.lang.acl.ACLMessage;

public interface ParentBehaviourCallback {

	
	public void callback(ACLMessage result, BehaviourCallbackItem arguments);
}
