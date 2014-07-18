package simulation.simulation;

import jade.core.AID;

public interface ISimulation {

	void notifyProductCreated(boolean succeeded, String productName, String equipletName);

	void notifyProductTraveling(String productName, String equipletName);

	void notifyProductFinished(String productName);
	
}
