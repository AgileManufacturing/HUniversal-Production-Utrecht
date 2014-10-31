package MAS.simulation.simulation;

public interface ISimulation {

	void notifyProductCreated(String productName, String equipletName);

	void notifyProductCreationFailed(String productName);
	
	void notifyProductOverdue(String productName);

	void notifyProductTraveling(String productName, String equipletName);

	void notifyProductProcessing(String productName, String equipletName, String service, int index);

	void notifyProductFinished(String productName);

	void notifyReconfigReady(String equipletName);

	void log(String info, String agent, String message);

}
