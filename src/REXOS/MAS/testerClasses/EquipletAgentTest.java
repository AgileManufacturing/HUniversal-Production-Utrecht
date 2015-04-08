package MAS.testerClasses;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.Date;

import util.configuration.ServerConfigurations;
import MAS.equiplet.EquipletAgent;


public class EquipletAgentTest {
	
	private static final String MAIN_PORT = ServerConfigurations.GS_PORT;
	private static final String CONTAINER_NAME = "EquipletAgent";
	private static final String MAIN_HOST = ServerConfigurations.GS_IP;
	private static final long serialVersionUID = 1L;
	
	public static void main(String[] args) throws StaleProxyException {
		Date date = new Date();
		// Spawning EquipletAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();
		profile.setParameter(Profile.MAIN_HOST, MAIN_HOST);
		profile.setParameter(Profile.MAIN_PORT, MAIN_PORT);
		profile.setParameter(Profile.CONTAINER_NAME, CONTAINER_NAME);

		AgentContainer container = runtime.createAgentContainer(profile);

		Object[] arguments = new Object[] { "hal" };
		AgentController ac = container.createNewAgent("EQ2", EquipletAgent.class.getName(), arguments);
		ac.start();
	}
}
