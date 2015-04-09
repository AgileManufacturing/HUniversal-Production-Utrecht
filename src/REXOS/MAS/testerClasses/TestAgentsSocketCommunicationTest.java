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

public class TestAgentsSocketCommunicationTest {
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
	 * @var CONTAINER_NAME The string holds the container name where in the TestReceiverAgent is being spawned.
	 */
	private static final String CONTAINER_NAME = "TestAgentsSpawnerAgent";

	private static final String baseName = "TA";

	/**
	 * main() Spawns the TestAgentSpawnerAgent on the selected server.
	 */
	public static void main(String[] args) throws StaleProxyException {
		spawnTestSpawnerAgent();
		spawnTestCommuncationTest("Socket");
	}
	public static void spawnTestSpawnerAgent() throws StaleProxyException{
		// Spawning TestRecieverAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();
		profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
		profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
		profile.setParameter(Profile.CONTAINER_NAME,  CONTAINER_NAME);
	
		AgentContainer container = runtime.createAgentContainer(profile);
	
		Object[] arguments = {baseName, 0, 500};
		String name = "TSA";
		AgentController ac = container.createNewAgent(name, TestAgentsSpawnerAgent.class.getName(), arguments);
		ac.start();
	}
	
	public static void spawnTestCommuncationTest(String type) throws StaleProxyException{
		// Spawning TestRecieverAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();
		profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
		profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
		profile.setParameter(Profile.CONTAINER_NAME,  CONTAINER_NAME);
	
		AgentContainer container = runtime.createAgentContainer(profile);
	
		Object[] arguments = {type, baseName, 0, 500, 5};
		String name = "TACTA";
		AgentController ac = container.createNewAgent(name, TestAgentCommunicationTesterAgent.class.getName(), arguments);
		ac.start();
	}
}
