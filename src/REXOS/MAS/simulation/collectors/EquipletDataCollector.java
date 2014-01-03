package simulation.collectors;

import java.util.HashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.mas_entities.Equiplet;

public class EquipletDataCollector extends DataCollector {
	HashMap<Equiplet, HashMap<Long, Equiplet.EquipletState>> equipletStates;
	private HashMap<Equiplet, HashMap<Long, Double>> equipletLoads;

	public EquipletDataCollector(Simulation simulation) {
		super(simulation);
		equipletStates = new HashMap<Equiplet, HashMap<Long, Equiplet.EquipletState>>(); 
		equipletLoads = new HashMap<Equiplet, HashMap<Long, Double>>(); 
	}
	
	
	public void collectData(long time) {
		if(needNewSample(time) == true) {
			Updatable[] updatables = this.simulation.getUpdatables();
			
			for (Updatable updatable : updatables) {
				if(updatable instanceof Equiplet) {
					processEquiplet((Equiplet) updatable);
				}
			}
		}
	}
	private void processEquiplet(Equiplet equiplet) {
		if(equipletStates.containsKey(equiplet) == false) {
			equipletStates.put(equiplet, new HashMap<Long, Equiplet.EquipletState>());
			getEquipletLoads().put(equiplet, new HashMap<Long, Double>());
		}
		equipletStates.get(equiplet).put(simulation.getCurrentSimulationTime(), equiplet.getEquipletState());
		getEquipletLoads().get(equiplet).put(simulation.getCurrentSimulationTime(), equiplet.getLoad());
	}


	public HashMap<Equiplet, HashMap<Long, Double>> getEquipletLoads() {
		return new HashMap<Equiplet, HashMap<Long, Double>>(equipletLoads);
	}
}
