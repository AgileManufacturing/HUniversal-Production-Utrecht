package SCADA;

import jade.core.AID;

public interface SCADABasicListener {
	 void onBasicUpdate(AID agent, String message);
}
