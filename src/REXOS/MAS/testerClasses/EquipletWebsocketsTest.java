package MAS.testerClasses;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.Date;
import java.util.concurrent.Delayed;

import util.configuration.ServerConfigurations;
import MAS.equiplet.EquipletAgent;

public class EquipletWebsocketsTest {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MAIN_HOST The string has the IP of the server that is hosting the Grid Agent Container.
	 */
	private static final String MAIN_HOST = ServerConfigurations.GS_IP;

	/**
	 * @var MAIN_PORT The string has the PORT of the server that is hosting the Grid Agent Container.
	 */
	private static final String MAIN_PORT = ServerConfigurations.GS_PORT;

	/**
	 * @var CONTAINER_NAME The string holds the container name where in the EquipletAgent is being spawned.
	 */
	private static final String CONTAINER_NAME = "EquipletAgent";

	/**
	 * main() Spawns the EquipletAgent on the selected server.
	 */
	public static void main(String[] args) throws Exception {
		spawnAgents(2, 500);
	}
	
	public static void spawnAgents(int startingNumberAgentName, int amountOfAgents) throws StaleProxyException{

		for(int i = startingNumberAgentName; i < amountOfAgents+startingNumberAgentName; i++){
			// Spawning EquipletAgent in the container that has the selected IP/Port
			jade.core.Runtime runtime = jade.core.Runtime.instance();
			Profile profile = new ProfileImpl();
			profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
			profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
			profile.setParameter(Profile.CONTAINER_NAME, CONTAINER_NAME);
		
			AgentContainer container = runtime.createAgentContainer(profile);
		
			Object[] arguments = new Object[] { "hal" };
			String name = "EQ" + i;
			AgentController ac = container.createNewAgent(name, EquipletAgent.class.getName(), arguments);
			ac.start();
			//add delay I suppose otherwise the agents will not be added.
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public static void sendMessages(int startingNumberAgentName, int amountOfAgents, int amountOfMessages){
		for(int i = 0; i < amountOfMessages; i++){
			for(int agentNumber = startingNumberAgentName; agentNumber < amountOfAgents + startingNumberAgentName; agentNumber++){
				//get agent.
				//execute send message.
			}
		}
	}
}
