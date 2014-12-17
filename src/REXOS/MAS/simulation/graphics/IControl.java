package MAS.simulation.graphics;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import MAS.util.Tick;
import MAS.util.Triple;

public interface IControl {

	void step();

	void start();

	void pause();

	int getDelay();

	void setDelay(int value);

	void saveStatistics();

	Map<String, List<Triple<String, Tick, Tick>>> getCompleteSchedule();

	Map<String, List<Triple<String, Tick, Tick>>> getEquipletSchedule();

	Map<String, List<Triple<String, Tick, Tick>>> getEquipletHistory();

	Map<String, Triple<? extends Number, ? extends Number, ? extends Number>> getEquipletUtilization();

	Map<String, Map<Tick, Tick>> getEquipletLatency();

	Map<String, TreeMap<Tick, Integer>> getProductStatistics();

	Map<? extends Number, ? extends Number> getProductionTimes();

	Map<String, Map<Tick, Float>> getEquipletStatistics();

	Map<String, Map<Tick, Float>> getEquipletLoadHistories();

}
