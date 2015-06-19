package SCADA;

import jade.core.AID;

public interface SCADADetailedListener {
	 void onDetailedUpdate(AID agent, String message);
}
