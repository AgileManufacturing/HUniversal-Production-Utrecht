package simulation.simulation;

public interface ISimulation {

	void notifyProductCreated(boolean succeeded, String productName, String equipletName);

	void notifyProductTraveling(String productName, String equipletName);
	
	void notifyProductProcessing(String productName, String equipletName, String service);

	void notifyProductFinished(String productName);
	
}
