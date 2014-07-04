package simulation.simulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import simulation.config.Config;
import simulation.mas.Equiplet;
import simulation.mas.Logistics;
import simulation.util.Position;

public class Grid {

	/*
	 * TODO
	 * add capabilities
	 * services
	 * distances
	 */

	private static Grid mGrid;
	private Map<String, Equiplet> equiplets;
	private Logistics logistics;
	private Stochastics stochastics;
	private double travelCost;

	public static Grid getInstance() {
		if (mGrid == null) {
			mGrid = new Grid();
		}
		return mGrid;
	}

	private Grid() {
		System.out.println("GRID: initiate");
		logistics = new Logistics();
		equiplets = new HashMap<String, Equiplet>();
		Config config = Config.read();
		stochastics = new Stochastics(config);
		travelCost = config.getTravelTime().first;

		// fill equiplets
		for (Equiplet equiplet : config.getEquipletList()) {
			equiplets.put(equiplet.getName(), equiplet);
		}

		System.out.println("GRID: config " + config);
		System.out.println("GRID: initialized");
	}

	/**
	 * 
	 * @return the logistics agent from the grid
	 */
	public Logistics getLogisticAgent() {
		return logistics;
	}

	/**
	 * Grid simulation variant of the DFService.search for equiplets providing
	 * the service
	 * 
	 * @param service
	 *            name
	 * @return list of equiplets that registed the service
	 */
	public List<Equiplet> serviceSearch(String service) {
		List<Equiplet> suitedEquiplets = new ArrayList<>();
		for (Entry<String, Equiplet> entry : equiplets.entrySet()) {
			Equiplet equiplet = entry.getValue();
			if (equiplet.providesService(service)) {
				suitedEquiplets.add(equiplet);
			}
		}
		return suitedEquiplets;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder("Grid [equiplets=");
		builder.append("\n[\n\t");
		for (Entry<String, Equiplet> equiplet : equiplets.entrySet()) {
			builder.append(equiplet.toString());
			builder.append(", \n\t");
		}
		builder.delete(builder.length() - 4, builder.length());
		builder.append("\n]");
		return builder.toString();
	}

	public Equiplet getEquiplet(String equiplet) {
		return equiplets.get(equiplet);
	}

	protected double getTravelTime(Position a, Position b) {
		int travelSquares = Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
		return stochastics.generateTravelTime(travelSquares);
	}

	protected Map<String, Equiplet> getEquiplets() {
		return equiplets;
	}

	public double getTravelCost() {
		return travelCost;
	}
}
