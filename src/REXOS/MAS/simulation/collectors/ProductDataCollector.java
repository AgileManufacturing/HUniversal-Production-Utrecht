package simulation.collectors;

import java.util.ArrayList;
import java.util.HashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.mas_entities.Product;

public class ProductDataCollector extends DataCollector {
	HashMap<Long, ArrayList<Product>> productCount;
	HashMap<Long, ArrayList<Product>> productsOverDeadline;

	public ProductDataCollector(Simulation simulation) {
		super(simulation);
		productCount = new HashMap<Long, ArrayList<Product>>(); 
		productsOverDeadline = new HashMap<Long, ArrayList<Product>>(); 
	}
	
	
	public void collectData(long time) {
		if(needNewSample(time) == true) {
			productCount.put(simulation.getCurrentSimulationTime(), new ArrayList<Product>());
			productsOverDeadline.put(simulation.getCurrentSimulationTime(), new ArrayList<Product>());
			Updatable[] updatables = this.simulation.getUpdatables();
			
			for (Updatable updatable : updatables) {
				if(updatable instanceof Product) {
					processProduct((Product) updatable);
				}
			}
		}
	}
	private void processProduct(Product product) {
		/*if(equipletStates.containsKey(equiplet) == false) {
			equipletStates.put(equiplet, new HashMap<Long, Equiplet.EquipletState>());
			getEquipletLoads().put(equiplet, new HashMap<Long, Double>());
		}*/
		productCount.get(simulation.getCurrentSimulationTime()).add(product);
		if(product.getDeadline() < simulation.getCurrentSimulationTime()) {
			productsOverDeadline.get(simulation.getCurrentSimulationTime()).add(product);
		}
	}


	public HashMap<Long, ArrayList<Product>> getProductCount() {
		return new HashMap<Long, ArrayList<Product>>(productCount);
	}
}
