package simulation.simulation;

public interface ISimulation {

	void notifyProductCreationFailed(String localName);
	
	void notifyProductCreated(String productName, String equipletName);

	void notifyProductTraveling(String productName, String equipletName);
	
	void notifyProductProcessing(String productName, String equipletName, String service);

	void notifyProductFinished(String productName);
	
}
