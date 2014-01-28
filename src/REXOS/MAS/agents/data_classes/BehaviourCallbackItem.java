package agents.data_classes;

import java.util.HashMap;

public class BehaviourCallbackItem {
	private HashMap<String, Object> arguments;
	
	public BehaviourCallbackItem(){
		arguments = new HashMap<String, Object>();
	}
	
	public BehaviourCallbackItem(HashMap<String, Object> arguments){
		this.arguments = arguments;
	}
	
	public void setArguments(HashMap<String, Object> newArguments){
		this.arguments = newArguments;
	}
	
	public HashMap<String, Object> getArguments(){
		return arguments;
	}
	
	public void addArgument(String key, Object value){
		arguments.put(key, value);
	}
	
	public Object getArgument(String key){
		return arguments.get(key);
	}
}
