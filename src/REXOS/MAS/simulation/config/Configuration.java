package MAS.simulation.config;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.json.JSONObject;

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.simulation.util.Settings;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.Tick;

public class Configuration implements IConfig {

	public static class ConfigException extends Exception {
		private static final long serialVersionUID = 1L;

		public ConfigException(String message) {
			super(message);
		}
	}

	// run length of the simulation
	private List<EquipletConfig> equipletConfig = new ArrayList<EquipletConfig>();
	private IConfig legacy;
	private List<ProductStep> productSteps;

	public Configuration(List<EquipletConfig> equiplets, List<ProductStep> steps, IConfig legacy) {
		this.equipletConfig = equiplets;
		this.productSteps = steps;
		this.legacy = legacy;
	}

	private static final String VALUE_SEPERATION = ";";
	private static final String CAPABILITY_SEPERATION = ",";

	public static Configuration read(IConfig legacy) throws ConfigException {
		List<EquipletConfig> equipletConfig = new ArrayList<EquipletConfig>();

		File file = new File(Settings.SIMULATION_EQUIPLET_CONFIG);
		System.out.println("Configuration " + file.getAbsolutePath());

		BufferedReader reader = null;

		try {
			String line = "";
			reader = new BufferedReader(new FileReader(file));
			while ((line = reader.readLine()) != null) {
				if (!line.startsWith("//") && line.matches(".+")) {
					String[] values = line.split(VALUE_SEPERATION);
					String name = values[0];
					PositionConfig position = parsePosition(values[1]);
					List<CapabilityConfig> capabilities = parseCapabilities(values[2]);
					DurationConfig breakdown = parseDuration(values[3]);
					DurationConfig repaired = parseDuration(values[4]);

					equipletConfig.add(new EquipletConfig(name, position, breakdown, repaired, capabilities));

					Collection<String> list = new ArrayList<String>();
					Collections.addAll(list, values);
					System.out.println("line = " + list);
				}
			}

		} catch (FileNotFoundException e) {
			throw new ConfigException("configuration file not found: " + e.getMessage());
		} catch (IOException e) {
			throw new ConfigException("configuration IOException: " + e.getMessage());
		} finally {
			if (reader != null) {
				try {
					reader.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}

		int counter = 1;
		List<ProductStep> steps = new ArrayList<ProductStep>();
		for (EquipletConfig equiplet : equipletConfig) {
			for (CapabilityConfig capability : equiplet.getCapabilities()) {
				steps.add(new ProductStep(counter++, capability.getName(), new JSONObject()));
			}
		}

		return new Configuration(equipletConfig, steps, legacy);
	}

	private static DurationConfig parseDuration(String source) throws ConfigException {
		Pattern pattern = Pattern.compile("(exp|normal|weibull|.?)\\(?(\\d+)\\)?");

		Matcher m = pattern.matcher(source);
		if (m.find()) {
			try {
				String type = m.group(1);
				double mean = Double.parseDouble(m.group(2));
				return new DurationConfig(type, mean);
			} catch (NumberFormatException e) {
				throw new ConfigException("parsing duration failed in \"" + source + "\" : " + e.getMessage());
			}
		} else {
			throw new ConfigException("parsing duration type failed \"" + source + "\"");
		}

	}

	private static PositionConfig parsePosition(String source) throws ConfigException {
		Pattern pattern = Pattern.compile("\\((\\d+),(\\d+)\\)");
		Matcher matcher = pattern.matcher(source);
		if (matcher.find()) {
			try {
				int x = Integer.parseInt(matcher.group(1));
				int y = Integer.parseInt(matcher.group(2));
				return new PositionConfig(x, y);
			} catch (NumberFormatException e) {
				throw new ConfigException("parsing position failed in \"" + source + "\" : " + e.getMessage());
			}
		} else {
			throw new ConfigException("parsing position failed \"" + source + "\"");
		}
	}

	private static List<CapabilityConfig> parseCapabilities(String source) throws ConfigException {
		Pattern pattern = Pattern.compile("(\\w+)=(.*)");

		List<CapabilityConfig> capabilities = new ArrayList<>();
		String[] rawCapabilities = source.split(CAPABILITY_SEPERATION);

		for (String capability : rawCapabilities) {

			Matcher m = pattern.matcher(capability);
			if (m.find()) {
				String service = m.group(1);
				DurationConfig duration = parseDuration(m.group(2));

				capabilities.add(new CapabilityConfig(service, duration, new HashMap<String, Object>()));
			}
		}
		return capabilities;
	}

	public Tick getRunLength() {
		return legacy.getRunLength();
	}

	public int getRuns() {
		return legacy.getRuns();
	}

	public Pair<Tick, DurationType> getTravelTime() {
		return legacy.getTravelTime();
	}

	public Pair<Tick, DurationType> getProductArrival() {
		return legacy.getProductArrival();
	}

	public Pair<Tick, DurationType> getProductDeadline() {
		return legacy.getProductDeadline();
	}

	public List<ProductStep> getProductSteps() {
		return productSteps;
	}

	public int getProductStepProbablity(ProductStep productStep) {
		return legacy.getProductStepProbablity(productStep);
	}

	public Map<String, Pair<Position, List<Capability>>> getEquipletsConfigurations() {
		Map<String, Pair<Position, List<Capability>>> equiplets = new HashMap<>();
		for (EquipletConfig e : equipletConfig) {
			Position position = new Position(e.getPosition().getX(), e.getPosition().getY());
			List<Capability> capabilities = new ArrayList<>();
			for (CapabilityConfig c : e.getCapabilities()) {
				//TODO Get services instead of c.getName()
				capabilities.add(new Capability(c.getName(), c.getName(), new HashMap<String, Object>(), new Tick(c.getDuration())));
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
		System.out.println("Config error: breakdown time of " + equiplet + " in " + toString());
		return null;
	}

	public Pair<Tick, DurationType> equipletRepaireTime(String equiplet) {
		for (EquipletConfig config : equipletConfig) {
			if (config.getName().equalsIgnoreCase(equiplet)) {
				return new Pair<Tick, DurationType>(new Tick(config.getRepaired().mean()), config.getRepaired().type());
			}
		}
		System.out.println("Config error: repair time of " + equiplet + " in " + toString());
		return null;
	}

	@Override
	public String toString() {
		return String.format("Config [run length=%s, runs=%d, travel=%s (%s), \n products=%s, \n equiplets=%s]", legacy.getRunLength(), legacy.getRuns(), legacy.getTravelTime().first, legacy.getTravelTime().second, legacy.getProductSteps(), equipletConfig);
	}
}
