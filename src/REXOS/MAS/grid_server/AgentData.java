package grid_server;

import java.util.ArrayList;

public class AgentData {
	private String name;
	private String address;
	private ArrayList<String> service;
	
	public AgentData(String name, String address, ArrayList<String> service){
		this.name = name;
		this.address = address;
		this.service = service;		
	}
	
	public String getName(){
		return name;
	}
	
	public String getAddress(){
		return address;
	}
	
	public ArrayList<String> getServices(){
		return service;
	}
}
