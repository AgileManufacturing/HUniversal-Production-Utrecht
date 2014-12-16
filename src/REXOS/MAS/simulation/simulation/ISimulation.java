package MAS.simulation.simulation;

import MAS.util.Tick;

public interface ISimulation {

	void notifyProductCreationFailed(String localName);

	void notifyProductCreated(String productName, String equipletName);

	void notifyProductCreationFailed(String productName);

	void notifyProductOverdue(String productName);

	void notifyProductTraveling(String productName, String equipletName);

	void notifyProductProcessing(String productName, String equipletName, String service, int index);

	void notifyProductFinished(String productName);

	void notifyProductShouldStart(String productName, Tick start, int index);

	void notifyProductRescheduled(String productName, String equipletName, boolean succeeded);

	void notifyProductRescheduled(boolean succeeded);

	void notifyReconfigReady(String equipletName);

	void log(String info, String agent, String message);


}
