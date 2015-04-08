package MAS.testerClasses;

import jade.core.AID;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.Date;
import java.util.concurrent.Delayed;

import util.configuration.ServerConfigurations;
import MAS.equiplet.EquipletAgent;
import MAS.util.MASConfiguration;
import MAS.util.Ontology;

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
	 * @var CONTAINER_NAME The string holds the container name where in the TestAgent is being spawned.
	 */
	private static final String CONTAINER_NAME = "TestAgent";
	
	/**
	 * @var CONTAINER_NAME The string holds the container name where in the TestReceiverAgent is being spawned.
	 */
	private static final String RECEIVER_CONTAINER_NAME = "TestReceiverAgent";


	/**
	 * main() Spawns the EquipletAgent on the selected server.
	 */
	public static void main(String[] args) throws StaleProxyException {
		spawnReceiver();
		spawnAgents(3, 2);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		sendMessages(3, 2, 5);
		System.out.println("Done!");
	}
	
	public static void spawnReceiver() throws StaleProxyException{
			// Spawning TestRecieverAgent in the container that has the selected IP/Port
			jade.core.Runtime runtime = jade.core.Runtime.instance();
			Profile profile = new ProfileImpl();
			profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
			profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
			profile.setParameter(Profile.CONTAINER_NAME,  RECEIVER_CONTAINER_NAME);
		
			AgentContainer container = runtime.createAgentContainer(profile);
		
			Object[] arguments = null;
			String name = "TRA";
			AgentController ac = container.createNewAgent(name, TestReceiverAgent.class.getName(), arguments);
			ac.start();
	}
	
	public static void spawnAgents(int startingNumberAgentName, int amountOfAgents) throws StaleProxyException{

		for(int i = startingNumberAgentName; i < amountOfAgents+startingNumberAgentName; i++){
			// Spawning TestAgents in the container that has the selected IP/Port
			jade.core.Runtime runtime = jade.core.Runtime.instance();
			Profile profile = new ProfileImpl();
			profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
			profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
			profile.setParameter(Profile.CONTAINER_NAME, CONTAINER_NAME);
		
			AgentContainer container = runtime.createAgentContainer(profile);
		
			Object[] arguments = null;
			String name = "TA" + i;
			AgentController ac = container.createNewAgent(name, TestAgent.class.getName(), arguments);
			ac.start();
			
			
			// add delay so the agents have time to be added.
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public static void sendMessages(int startingNumberAgentName, int amountOfAgents, int amountOfMessages){
		for(int i = 0; i < amountOfMessages; i++){
			for(int agentNumber = startingNumberAgentName; agentNumber < amountOfAgents + startingNumberAgentName; agentNumber++){
				// get agent.
				String name = "TA" + agentNumber;
				AID agent = new AID(name, AID.ISLOCALNAME);
				// send message.
				String JSONMessage = "";
				ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
				message.addReceiver(agent);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setContent(JSONMessage);
				//send(message);
				System.out.println(agentNumber);
			}
		}
	}
}
