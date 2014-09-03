package simulation.graphics;

import java.util.List;
import java.util.Map;

import simulation.util.Triple;

public interface Control {

	void step();
	
	void start();

	void pause();

	int getDelay();

	void setDelay(int value);

	void saveStatistics();

	Map<String, List<Triple<String, Double, Double>>> getCompleteSchedule();
	
	Map<String, List<Triple<String, Double, Double>>> getEquipletSchedule();
	
	Map<String, List<Triple<String, Double, Double>>> getEquipletHistory();
	
	Map<String, Triple<Double, Double, Double>> getEquipletUtilization();

	Map<String, Map<Double, Double>> getEquipletLatency();

	Map<String, Map<Double, Double>> getProductStatistics();

	Map<String, Map<Double, Double>> getEquipletStatistics();

}
