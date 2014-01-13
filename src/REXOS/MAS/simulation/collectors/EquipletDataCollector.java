package simulation.collectors;

import java.util.Arrays;
import java.util.HashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.mas_entities.Equiplet;
import simulation.mas_entities.Equiplet.EquipletState;
import simulation.mas_entities.Grid;

public class EquipletDataCollector extends DataCollector {
	private HashMap<Long, HashMap<Equiplet, EquipletState>> equipletStates;
	private HashMap<Long, HashMap<Equiplet, Double>> equipletLoads;
	private HashMap<Long, HashMap<Equiplet, Double>> equipletCurrentLoads;

	public EquipletDataCollector(Simulation simulation) {
		super(simulation);
		equipletStates = new HashMap<Long, HashMap<Equiplet, Equiplet.EquipletState>>(); 
		equipletLoads = new HashMap<Long, HashMap<Equiplet, Double>>(); 
		equipletCurrentLoads = new HashMap<Long, HashMap<Equiplet, Double>>(); 
	}
	
	
	public void collectData(long time) {
		if(needNewSample(time) == true) {
			Updatable[] updatables = this.simulation.getUpdatables();
			
			equipletStates.put(time, new HashMap<Equiplet, Equiplet.EquipletState>());
			equipletLoads.put(time, new HashMap<Equiplet, Double>());
			equipletCurrentLoads.put(time, new HashMap<Equiplet, Double>());
			
			Grid grid = null;
			for (Updatable updatable : updatables) {
				if(updatable instanceof Grid) {
					grid = (Grid) updatable;
				}
			}
			if(grid == null) {
				// no grid?
				return;
			}
			for (Equiplet equiplet : grid.getEquiplets()) {
				processEquiplet(equiplet, time);
			}
		}
	}
	private void processEquiplet(Equiplet equiplet, long time) {
		equipletLoads.get(time).put(equiplet, equiplet.getLoad());
		equipletCurrentLoads.get(time).put(equiplet, equiplet.getCurrentLoad());
		equiplet.resetCurrentLoad();
		equipletStates.get(time).put(equiplet, equiplet.getEquipletState());
	}


	public HashMap<Long, HashMap<Equiplet, Double>> getEquipletLoads() {
		return new HashMap<Long, HashMap<Equiplet, Double>>(equipletLoads);
	}
	public HashMap<Long, HashMap<Equiplet, Double>> getEquipletCurrentLoads() {
		return new HashMap<Long, HashMap<Equiplet, Double>>(equipletCurrentLoads);
	}
	public HashMap<Long, HashMap<Equiplet, EquipletState>> getEquipletState() {
		return new HashMap<Long, HashMap<Equiplet, EquipletState>>(equipletStates);
	}
	
	/*public double[] getLoadForEquiplet(Equiplet subject, int resolution) {
		HashMap<Long, Double> equipletLoad = equipletLoads.get(subject);
		Long[] entries = equipletLoad.keySet().toArray(new Long[equipletLoad.size()]);
		Arrays.sort(entries);
		try {
			return getLoadForEquiplet(subject, resolution, entries[0], entries[entries.length - 1]);
		} catch (Exception e) {
			// should NEVER happen
			e.printStackTrace();
		}
		return null; // dead code, fucking java crap shit
	}
	public double[] getLoadForEquiplet(Equiplet subject, int resolution, long startTime, long endTime) throws Exception {
		HashMap<Long, Double> equipletLoad = equipletLoads.get(subject);
		Long[] entries = equipletLoad.keySet().toArray(new Long[equipletLoad.size()]);
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
			output[i] = equipletLoad.get(leftIndex) * leftWeight + equipletLoad.get(rightIndex) * rightWeight; 
		}
		return output;
	}*/
	/*public Equiplet.EquipletState[] getEquipletStateForEquiplet(Equiplet subject, int resolution) {
		HashMap<Long, Equiplet.EquipletState> equipletState = equipletStates.get(subject);
		Long[] entries = equipletState.keySet().toArray(new Long[equipletState.size()]);
		Arrays.sort(entries);
		try {
			return getEquipletStateForEquiplet(subject, resolution, entries[0], entries[entries.length - 1]);
		} catch (Exception e) {
			// should NEVER happen
			e.printStackTrace();
		}
		return null; // dead code, fucking java crap shit
		
	}
	public Equiplet.EquipletState[] getEquipletStateForEquiplet(Equiplet subject, int resolution, long startTime, long endTime) throws Exception {
		HashMap<Long, Equiplet.EquipletState> equipletState = equipletStates.get(subject);
		Long[] entries = equipletStates.keySet().toArray(new Long[equipletStates.size()]);
		Arrays.sort(entries);
		
		if(startTime < entries[0] || endTime > entries[entries.length - 1]) {
			throw new Exception("startTime < entries[0] || endTime > entries[entries.length - 1]");
		} else if(resolution == 0) {
			// stupid troll
			return new Equiplet.EquipletState[0];
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
		
		Equiplet.EquipletState[] output = new Equiplet.EquipletState[resolution];
		double resolutionRangeFactor = entriesInRange / (double) resolution;
		for(int i = 0; i < resolution; i++) {
			double virtualIndex = i * resolutionRangeFactor;
			long index = (long) Math.round(virtualIndex);
			output[i] = equipletState.get(index); 
		}
		return output;
	}*/
}
