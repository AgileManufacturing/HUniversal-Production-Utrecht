package simulation.simulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import simulation.config.Config;
import simulation.mas.Logistics;
import simulation.mas.equiplet.IEquipletSim;
import simulation.util.Position;

public class Grid {

	/*
	 * TODO
	 * add capabilities
	 * services
	 * distances
	 */

	private static Grid mGrid;
	private Map<String, IEquipletSim> equiplets;
	
	public static Grid getInstance() {
		if (mGrid == null) {
			mGrid = new Grid();
		}
		return mGrid;
	}

	private Grid() {
		System.out.println("GRID: initiate");
		equiplets = new HashMap<String, IEquipletSim>();

		System.out.println("GRID: initialized");
	}

	public void registerEquiplets(String name, IEquipletSim equiplet) {
		equiplets.put(name, equiplet);
	}

	/**
	 * Grid simulation variant of the DFService.search for equiplets providing
	 * the service
	 * 
	 * @param service
	 *            name
	 * @return list of equiplets that registed the service
	 */
	public List<IEquipletSim> serviceSearch(String service) {
		List<IEquipletSim> suitedEquiplets = new ArrayList<>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			IEquipletSim equiplet = entry.getValue();
			//if (equiplet.(service)) {
				//suitedEquiplets.add(equiplet);
			//}
		}
		return suitedEquiplets;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder("Grid [equiplets=");
		builder.append("\n[\n\t");
		for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
			builder.append(equiplet.toString());
			builder.append(", \n\t");
		}
		builder.delete(builder.length() - 4, builder.length());
		builder.append("\n]");
		return builder.toString();
	}

	public IEquipletSim getEquiplet(String equiplet) {
		return equiplets.get(equiplet);
	}

	protected Map<String, IEquipletSim> getEquiplets() {
		return equiplets;
	}

	public void killAgent(String name) {
		// TODO Auto-generated method stub
		
	}
}
