package simulation.mas.equiplet;

import java.util.List;
import java.util.Map;

import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public interface IEquipletSim {

	Position getPosition();

	EquipletState getEquipletState();

	double load(double time, double window);

	void notifyJobFinished(double time);

	void notifyBreakdown(double time);

	void notifyRepaired(double time);

	double getRemainingTime();

	Triple<Double, Double, Double> getStatistics(double time);

	List<Triple<String, Double, Double>> getCompleteSchedule();

	List<Triple<String, Double, Double>> getSchedule();

	List<Triple<String, Double, Double>> getHistory();

	Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> getUpdateState();

	Map<Double, Double> getLatency();

}
