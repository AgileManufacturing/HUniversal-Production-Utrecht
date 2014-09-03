package simulation.mas.equiplet;

import java.util.List;
import java.util.Map;

import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public class EquipletSim extends Equiplet implements IEquipletSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String name;

	public EquipletSim(String equipletName, Position position, List<Capability> capabilities) {
		this.name = equipletName;
		init(position, capabilities);
	}

	public String getEquipletName() {
		return name;
	}

	@Override
	public Position getPosition() {
		return position;
	}

	@Override
	public EquipletState getEquipletState() {
		return state;
	}

	@Override
	public double load(double time, double window) {
		return super.load(time, window);
	}

	@Override
	public void notifyJobFinished(double time) {
		// TODO Auto-generated method stub

	}

	@Override
	public void notifyBreakdown(double time) {
		// TODO Auto-generated method stub

	}

	@Override
	public void notifyRepaired(double time) {
		// TODO Auto-generated method stub

	}

	@Override
	public double getRemainingTime() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Triple<Double, Double, Double> getStatistics(double time) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Triple<String, Double, Double>> getCompleteSchedule() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Triple<String, Double, Double>> getSchedule() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Triple<String, Double, Double>> getHistory() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> getUpdateState() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Map<Double, Double> getLatency() {
		// TODO Auto-generated method stub
		return null;
	}

}
