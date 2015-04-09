package MAS.testerClasses;

import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import util.configuration.ServerConfigurations;

public class TestAgentsSpawnerAgent extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MAIN_HOST The string has the IP of the server that is hosting the
	 *      Grid Agent Container.
	 */
	private static final String MAIN_HOST = ServerConfigurations.GS_IP;

	/**
	 * @var MAIN_PORT The string has the PORT of the server that is hosting the
	 *      Grid Agent Container.
	 */
	private static final String MAIN_PORT = ServerConfigurations.GS_PORT;

	/**
	 * @var CONTAINER_NAME The string holds the container name where in the
	 *      TestAgent is being spawned.
	 */
	private static final String CONTAINER_NAME = "TestAgent";

	/**
	 * @var CONTAINER_NAME The string holds the container name where in the
	 *      TestReceiverAgent is being spawned.
	 */
	private static final String RECEIVER_CONTAINER_NAME = "TestReceiverAgent";

	/**
	 * main() Spawns the TestAgents on the grid.
	 */
	protected void setup() {
		Object[] args = getArguments();
		if(args.length >= 3){
			String agentBaseName = args[0].toString(); 
			int startValueAgents = (int) args[1];
			int amountOfAgents = (int) args[2];
			if(!agentBaseName.equals("") && startValueAgents >= 0 && amountOfAgents > 0){
				try {
					spawnReceiver();
					spawnAgents(agentBaseName, startValueAgents, amountOfAgents);
					Thread.sleep(1000);
					System.out.println("Done Spawning!");
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (StaleProxyException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}else{
				System.err.println("Invalid argument agent: " + getLocalName());
			}
		}else{
			System.err.println("Missing arguments agent: " + getLocalName());
		}
	}

	public void spawnReceiver() throws StaleProxyException {
		// Spawning TestRecieverAgent in the container that has the selected
		// IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();
		profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
		profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
		profile.setParameter(Profile.CONTAINER_NAME, RECEIVER_CONTAINER_NAME);

		AgentContainer container = runtime.createAgentContainer(profile);

		Object[] arguments = null;
		String name = "TRA";
		AgentController ac = container.createNewAgent(name,
				TestReceiverAgent.class.getName(), arguments);
		ac.start();
	}

	public void spawnAgents(String baseAgentName, int startingNumberAgentName, int amountOfAgents)
			throws StaleProxyException {

		for (int i = startingNumberAgentName; i < amountOfAgents
				+ startingNumberAgentName; i++) {
			// Spawning TestAgents in the container that has the selected
			// IP/Port
			jade.core.Runtime runtime = jade.core.Runtime.instance();
			Profile profile = new ProfileImpl();
			profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
			profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
			profile.setParameter(Profile.CONTAINER_NAME, CONTAINER_NAME);

			AgentContainer container = runtime.createAgentContainer(profile);

			Object[] arguments = null;
			AgentController ac = container.createNewAgent(baseAgentName, TestAgent.class.getName(), arguments);
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

}
