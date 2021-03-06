package MAS.simulation.mas.equiplet;

import java.util.List;
import java.util.Map;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletState;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;

public interface IEquipletSim {

	Position getPosition();

	EquipletState getEquipletState();

	List<Capability> getCapabilities();

	double load(Tick time, Tick window);

	double loadHistory(Tick time, Tick window);

	void notifyJobFinished(Tick time);

	void notifyBreakdown(Tick time);

	void notifyRepaired(Tick time);

	Tick getRemainingTime();

	Triple<Tick, Tick, Tick> getStatistics(Tick time);

	List<Triple<String, Tick, Tick>> getCompleteSchedule();

	List<Triple<String, Tick, Tick>> getSchedule();

	List<Triple<String, Tick, Tick>> getHistory();

	Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> getUpdateState();

	Map<Tick, Tick> getLatency();

	void kill();

	void reconfigureStart();

	void notifyReconfigured(List<Capability> capabilities);
	
	boolean isExecuting();
	
	String getExecutingProduct();
	
	String toFullString();
}
