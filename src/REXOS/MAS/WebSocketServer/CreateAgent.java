package WebSocketServer;

import com.google.gson.JsonObject;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentController;
import jade.wrapper.ControllerException;


public class CreateAgent {
	/**
	  * @var MAIN_HOST
	  * The string has the IP of the server that is hosting the Grid Agent Container.
	  */
	private static final String MAIN_HOST = "145.89.126.159";
	
	/**
	  * @var MAIN_PORT
	  * The string has the PORT of the server that is hosting the Grid Agent Container.
	  */
	private static final String MAIN_PORT = "1234";
	
	/**
	  * @var CONTAINER_NAME
	  * The string holds the container name where in the EquipletAgent is being spawned.
	  */
	private static final String CONTAINER_NAME = "ProductAgentSpawnerAgent";

	
	 /**
	  * main()
	  * Spawns the EquipletAgent on the selected server.
	 * @throws ControllerException 
	  */
	public void createAgent(String args) throws ControllerException{
		java.util.Date date= new java.util.Date();
		//Spanwing EquipletAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();		
		profile.setParameter(Profile.MAIN_HOST,MAIN_HOST);
		profile.setParameter(Profile.MAIN_PORT,MAIN_PORT);
		profile.setParameter(Profile.CONTAINER_NAME,CONTAINER_NAME+date.getTime());
		jade.wrapper.AgentContainer container = runtime.createAgentContainer( profile );
		ProductAgentSpawnerAgent agent = new ProductAgentSpawnerAgent();
		agent.setProductSteps(args);
		try{
		
			AgentController ac = container.acceptNewAgent( container.getContainerName(), agent);
			ac.start();
		
		}catch(ControllerException e){
			AgentController ac = container.acceptNewAgent( container.getContainerName()+"12", agent);
			ac.start();
		}
	}
}
