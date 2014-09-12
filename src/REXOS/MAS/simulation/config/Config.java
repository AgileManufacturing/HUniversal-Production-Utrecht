package simulation.config;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlElementWrapper;
import javax.xml.bind.annotation.XmlRootElement;

import simulation.mas.equiplet.Capability;
import simulation.mas.product.ProductStep;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Tick;

/**
 * Configuration data to read from configuration file simulation.xml
 * 
 * @author Laurens van den Brink
 * 
 */
@XmlRootElement
@XmlAccessorType(XmlAccessType.FIELD)
public class Config implements IConfig {

	// run length of the simulation
	@XmlElement(name = "runlength")
	private double runlength;

	// number of times the simulation needs to run
	@XmlElement(name = "runs")
	private int runs;

	@XmlElement(name = "travel")
	protected DurationConfig travel;

	@XmlElement(name = "products")
	protected ProductsConfig products;

	private List<EquipletConfig> equipletConfig = new ArrayList<EquipletConfig>();

	public static Config read() {
		try {
			File file = new File("Simulation.xml");
			JAXBContext jaxbContext = JAXBContext.newInstance(Config.class);

			Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();
			return (Config) jaxbUnmarshaller.unmarshal(file);

		} catch (JAXBException e) {
			e.printStackTrace();
			System.err.println("Failed to load config settings");
			return null;
		}
	}

	public Tick getRunLength() {
		return new Tick(runlength);
	}

	public int getRuns() {
		return runs;
	}

	public Pair<Tick, DurationType> getTravelTime() {
		return new Pair<Tick, DurationType>(new Tick(travel.mean()), travel.type());
	}

	public Pair<Tick, DurationType> getProductArrival() {
		return new Pair<Tick, DurationType>(new Tick(products.getArrival()), products.getArrivalType());
	}

	@Override
	public Pair<Tick, DurationType> getProductDeadline() {
		return new Pair<Tick, DurationType>(new Tick(products.getDeadline()), products.getDeadlineType());
	}

	public List<ProductStep> getProductSteps() {
		ArrayList<ProductStep> productSteps = new ArrayList<>();

		for (ProductStepConfig i : products.getSteps()) {
			productSteps.add(new ProductStep(productSteps.size(), i.getService(), i.getCriteria()));
		}

		return productSteps;
	}

	public int getProductStepProbablity(ProductStep productStep) {
		for (ProductStepConfig step : products.getSteps()) {
			if (step.getService().equalsIgnoreCase(productStep.getService())) {
				return step.getProbability();
			}
		}
		System.err.println("Config error: product step probablity of " + productStep + " in " + toString());
		return 0;
	}

	/**
	 * Set the equiplet data read from the configuration file
	 * 
	 * @param data
	 *            of the equiplet
	 */
	@XmlElementWrapper(name = "equiplets")
	@XmlElement(name = "equiplet", type = EquipletConfig.class)
	public void setEquiplets(List<EquipletConfig> data) {
		this.equipletConfig = data;
	}

	public List<EquipletConfig> getEquiplets() {
		return equipletConfig;
	}

	public Map<String, Pair<Position, List<Capability>>> getEquipletsConfigurations() {
		Map<String, Pair<Position, List<Capability>>> equiplets = new HashMap<>();
		for (EquipletConfig e : equipletConfig) {
			Position position = new Position(e.getPosition().getX(), e.getPosition().getY());
			List<Capability> capabilities = new ArrayList<>();
			for (CapabilityConfig c : e.getCapabilities()) {
				capabilities.add(new Capability(c.getName(), new HashMap<String, Object>(), equipletProductionTime(e.getName(), c.getName()).first));
			}

			equiplets.put(e.getName(), new Pair<Position, List<Capability>>(position, capabilities));
		}
		return equiplets;
	}

	public Pair<Tick, DurationType> equipletProductionTime(String equiplet, String service) {
		for (EquipletConfig config : equipletConfig) {
			if (config.getName().equalsIgnoreCase(equiplet)) {
				for (CapabilityConfig capability : config.getCapabilities()) {
					if (capability.getName().equalsIgnoreCase(service)) {
						return new Pair<Tick, DurationType>(new Tick(capability.getDuration()), capability.getDurationType());
					}
				}
			}
		}
		System.err.println("Config error: production time of " + equiplet + " " + service + " in " + toString());
		return null;
	}

	public Pair<Tick, DurationType> equipletBreakdownTime(String equiplet) {
		for (EquipletConfig config : equipletConfig) {
			if (config.getName().equalsIgnoreCase(equiplet)) {
				return new Pair<Tick, DurationType>(new Tick(config.getBreakdown().mean()), config.getBreakdown().type());
			}
		}
		System.err.println("Config error: breakdown time of " + equiplet + " in " + toString());
		return null;
	}

	public Pair<Tick, DurationType> equipletRepaireTime(String equiplet) {
		for (EquipletConfig config : equipletConfig) {
			if (config.getName().equalsIgnoreCase(equiplet)) {
				return new Pair<Tick, DurationType>(new Tick(config.getRepaired().mean()), config.getRepaired().type());
			}
		}
		System.err.println("Config error: repair time of " + equiplet + " in " + toString());
		return null;
	}

	@Override
	public String toString() {
		return String.format("Config [run length=%s, runs=%d, travel=%.2f (%s), \n products=%s, \n equiplets=%s]", runlength, runs, travel.mean(), travel.type(), products, equipletConfig);
	}
}
