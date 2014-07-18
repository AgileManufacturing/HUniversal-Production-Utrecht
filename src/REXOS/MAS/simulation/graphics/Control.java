package simulation.graphics;

import java.util.List;
import java.util.Map;

import simulation.mas.Equiplet;
import simulation.mas.Product;
import simulation.util.Triple;

public interface Control {

	Map<String, Product> getProducts();

	List<Equiplet> getEquiplets();

	Map<String, Triple<Double, Double, Double>> getEquipletHistory();

	void start();

	void pause();

	int getDelay();

	void setDelay(int value);

	void saveStatistics();

}
