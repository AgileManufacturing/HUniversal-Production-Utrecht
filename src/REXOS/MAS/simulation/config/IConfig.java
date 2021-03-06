package MAS.simulation.config;

import java.util.List;
import java.util.Map;

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.Tick;

public interface IConfig {

	public Tick getRunLength();

	public int getRuns();

	public Pair<Tick, DurationType> getTravelTime();

	public Pair<Tick, DurationType> getProductArrival();
	
	public Pair<Tick, DurationType> getProductDeadline();

	public List<ProductStep> getProductSteps();

	public int getProductStepProbablity(ProductStep productStep);

	public Map<String, Pair<Position, List<Capability>>> getEquipletsConfigurations();

	public Pair<Tick, DurationType> equipletProductionTime(String equiplet, String service);

	public Pair<Tick, DurationType> equipletBreakdownTime(String equiplet);

	public Pair<Tick, DurationType> equipletRepaireTime(String equiplet);

}