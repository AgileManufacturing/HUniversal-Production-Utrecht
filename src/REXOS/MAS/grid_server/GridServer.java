package grid_server;

import jade.core.Agent;


public class GridServer extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public static void main(String[] args) {
		//-gui -port 1234 -platform-id Platform2 -agents "Grid:agents.GridAgent;PartsAgent:agents.PartsAgent; MonitoringAgent:agents.MonitoringAgent"
		String [] argu = new String[2];
		argu[0] = "-gui"; 
		argu[1] = "Grid:grid_server.GridAgent;SupplyAgent:grid_server.SupplyAgent; MonitoringAgent:grid_server.MonitoringAgent"; 
		jade.Boot.main(argu);
	}

}
