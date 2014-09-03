package simulation.config;

import java.util.List;
import java.util.Map;

import simulation.mas.equiplet.Capability;
import simulation.mas.product.ProductStep;
import simulation.util.Pair;
import simulation.util.Position;

public interface IConfig {

	public double getRunLength();
	
	public int getRuns();

	public Pair<Double, DurationType> getTravelTime();

	public Pair<Double, DurationType> getProductArrival();

	public List<ProductStep> getProductSteps();

	public int getProductStepProbablity(ProductStep productStep);
	
	public Map<String, Pair<Position, List<Capability>>> getEquipletsConfigurations();
	
	public Pair<Double, DurationType> equipletProductionTime(String equiplet, String service);

	public Pair<Double, DurationType> equipletBreakdownTime(String equiplet);

	public Pair<Double, DurationType> equipletRepaireTime(String equiplet);

}