package simulation.collectors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.data.ProductStep;
import simulation.data.Schedule;
import simulation.mas_entities.Equiplet;
import simulation.mas_entities.Product;
import simulation.mas_entities.Product.ProductState;

public class ProductDataCollector extends DataCollector {
	public HashMap<Long, ArrayList<Product>> products;
	public HashMap<Long, ArrayList<Product>> productInProgres;
	public HashMap<Long, ArrayList<Product>> productsOverDeadline;
	public HashMap<Product, HashMap<Long, LinkedHashMap<ProductStep, Schedule>>> productSchedules;

	public ProductDataCollector(Simulation simulation) {
		super(simulation);
		products = new HashMap<Long, ArrayList<Product>>(); 
		productInProgres = new HashMap<Long, ArrayList<Product>>(); 
		productsOverDeadline = new HashMap<Long, ArrayList<Product>>();
		productSchedules = new HashMap<Product, HashMap<Long, LinkedHashMap<ProductStep, Schedule>>>();
	}
	
	
	public void collectData(long time) {
		if(needNewSample(time) == true) {
			products.put(simulation.getCurrentSimulationTime(), new ArrayList<Product>());
			productInProgres.put(simulation.getCurrentSimulationTime(), new ArrayList<Product>());
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
		products.get(simulation.getCurrentSimulationTime()).add(product);
		if(product.getState() == ProductState.failed) {
			productsOverDeadline.get(simulation.getCurrentSimulationTime()).add(product);
		} else if (product.getState() == ProductState.inProgress) {
			productInProgres.get(simulation.getCurrentSimulationTime()).add(product);
		}
		
		LinkedHashMap<ProductStep, Schedule> currentSchedule = product.getFinalSchedules();
		if(productSchedules.containsKey(product) == false) {
			productSchedules.put(product, new HashMap<Long, LinkedHashMap<ProductStep, Schedule>>());
			productSchedules.get(product).put(
					simulation.getCurrentSimulationTime(), (LinkedHashMap<ProductStep, Schedule>) currentSchedule.clone());
		}
		else {
			Long[] previousScheduleTimes = productSchedules.get(product).keySet().toArray(
					new Long[productSchedules.get(product).size() - 1]);
			Arrays.sort(previousScheduleTimes);
			LinkedHashMap<ProductStep, Schedule> previousSchedule = productSchedules.get(product).get(
					previousScheduleTimes[previousScheduleTimes.length - 1]);
			
			// are the schedule equal?
			if(compareSchedules(currentSchedule, previousSchedule) == false) {
				
				productSchedules.get(product).put(
						simulation.getCurrentSimulationTime(), (LinkedHashMap<ProductStep, Schedule>) currentSchedule.clone());
			}
		}
		
		
	}
	private boolean compareSchedules(LinkedHashMap<ProductStep, Schedule> scheduleA, LinkedHashMap<ProductStep, Schedule> scheduleB) {
		if(scheduleA.size() != scheduleB.size()) {
			return false;
		}
		for (Schedule schedule : scheduleA.values()) {
			if(scheduleB.values().contains(schedule)) {
				return false;
			}
		}
		return true;
	}


	public HashMap<Long, ArrayList<Product>> getProductCount() {
		return new HashMap<Long, ArrayList<Product>>(products);
	}
	public double[] getProductCount(int resolution) {
		Long[] entries = products.keySet().toArray(new Long[products.size()]);
		Arrays.sort(entries);
		try {
			return getProductCount(resolution, entries[0], entries[entries.length - 1]);
		} catch (Exception e) {
			// should NEVER happen
			e.printStackTrace();
		}
		return null; // dead code, fucking java crap shit
	}
	public double[] getProductCount(int resolution, long startTime, long endTime) throws Exception {
		Long[] entries = products.keySet().toArray(new Long[products.size()]);
		Arrays.sort(entries);
		
		if(startTime < entries[0] || endTime > entries[entries.length - 1]) {
			throw new Exception("startTime < entries[0] || endTime > entries[entries.length - 1]");
		} else if(resolution == 0) {
			// stupid troll
			return new double[0];
		}
		
		// find first entry
		int firstEntryIndex;
		for(firstEntryIndex = 0; firstEntryIndex < entries.length; firstEntryIndex++) {
			if(startTime >= entries[firstEntryIndex]) break;
		}
		// find last entry
		int lastEntryIndex;
		for(lastEntryIndex = entries.length - 1; lastEntryIndex >= 0; lastEntryIndex--) {
			if(endTime <= entries[lastEntryIndex]) break;
		}
		int entriesInRange = lastEntryIndex - firstEntryIndex;
		
		double[] output = new double[resolution];
		double resolutionRangeFactor = entriesInRange / (double) resolution;
		for(int i = 0; i < resolution; i++) {
			double virtualIndex = i * resolutionRangeFactor;
			long leftIndex = (long) Math.floor(virtualIndex);
			double leftWeight = Math.ceil(virtualIndex) - virtualIndex; 
			long rightIndex = (long) Math.ceil(virtualIndex);
			double rightWeight = virtualIndex - Math.floor(virtualIndex);
			if(leftIndex == rightIndex) {
				// we matched the exact same index, and the weight will be 0. compensate...
				leftWeight = 1;
			}
			output[i] = products.get(leftIndex).size() * leftWeight + products.get(rightIndex).size() * rightWeight; 
		}
		return output;
	}
	public double[] getProductsOverDeadline(int resolution) {
		Long[] entries = productsOverDeadline.keySet().toArray(new Long[productsOverDeadline.size()]);
		Arrays.sort(entries);
		try {
			return getProductsOverDeadline(resolution, entries[0], entries[entries.length - 1]);
		} catch (Exception e) {
			// should NEVER happen
			e.printStackTrace();
		}
		return null; // dead code, fucking java crap shit
	}
	public double[] getProductsOverDeadline(int resolution, long startTime, long endTime) throws Exception {
		Long[] entries = productsOverDeadline.keySet().toArray(new Long[productsOverDeadline.size()]);
		Arrays.sort(entries);
		
		if(startTime < entries[0] || endTime > entries[entries.length - 1]) {
			throw new Exception("startTime < entries[0] || endTime > entries[entries.length - 1]");
		} else if(resolution == 0) {
			// stupid troll
			return new double[0];
		}
		
		// find first entry
		int firstEntryIndex;
		for(firstEntryIndex = 0; firstEntryIndex < entries.length; firstEntryIndex++) {
			if(startTime >= entries[firstEntryIndex]) break;
		}
		// find last entry
		int lastEntryIndex;
		for(lastEntryIndex = entries.length - 1; lastEntryIndex >= 0; lastEntryIndex--) {
			if(endTime <= entries[lastEntryIndex]) break;
		}
		int entriesInRange = lastEntryIndex - firstEntryIndex;
		
		double[] output = new double[resolution];
		double resolutionRangeFactor = entriesInRange / (double) resolution;
		for(int i = 0; i < resolution; i++) {
			double virtualIndex = i * resolutionRangeFactor;
			long leftIndex = (long) Math.floor(virtualIndex);
			double leftWeight = Math.ceil(virtualIndex) - virtualIndex; 
			long rightIndex = (long) Math.ceil(virtualIndex);
			double rightWeight = virtualIndex - Math.floor(virtualIndex);
			if(leftIndex == rightIndex) {
				// we matched the exact same index, and the weight will be 0. compensate...
				leftWeight = 1;
			}
			output[i] = productsOverDeadline.get(leftIndex).size() * leftWeight + productsOverDeadline.get(rightIndex).size() * rightWeight; 
		}
		return output;
	}

}
