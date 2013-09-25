package gatewayserver;



import jade.cli.CLICommand;
import jade.cli.CLIManager;
import jade.core.ContainerID;
import jade.core.Specifier;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.domain.FIPAException;
import jade.domain.FIPAService;
import jade.lang.acl.ACLMessage;

import java.util.Properties;
import java.util.Vector;

public class CommandAgent extends CLICommand {

	private String json = "";
	private String productAgentID;
	
	public CommandAgent(String json, String productAgentID) {
		this.json = json;
		this.productAgentID = productAgentID;
	}
	
	

	@Override
	public Behaviour getBehaviour(final Properties pp) throws IllegalArgumentException {
		return new OneShotBehaviour(null) {
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			@Override
			public void action() {
				jade.domain.JADEAgentManagement.CreateAgent ca = new jade.domain.JADEAgentManagement.CreateAgent();
				ca.setAgentName(productAgentID);
				ca.setClassName("agents.product_agent.ProductAgent");
				ca.setContainer(new ContainerID("Main-Container", null));
				ca.addArguments(json);
				
				
				String argumentsList = pp.getProperty("arguments");
				Vector<?> v = Specifier.parseList(argumentsList, ',');
				for (Object a : v) {
					ca.addArguments(a);
				}
				
				try {
					ACLMessage request = CLIManager.createAMSRequest(myAgent, ca);
					ACLMessage response = FIPAService.doFipaRequestClient(myAgent, request, 1000000);
					if (response == null) {
						System.out.println("Timeout expired");
					}
				}
				catch (FIPAException fe) {
					System.out.println(fe.getMessage());
				}
				catch (Exception e) {
					System.out.println(e.getMessage());
				}
			}
			
		};
	}

}