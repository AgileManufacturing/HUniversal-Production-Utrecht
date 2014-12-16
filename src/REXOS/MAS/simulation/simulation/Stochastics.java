package MAS.simulation.simulation;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.simulation.config.DurationType;
import MAS.simulation.config.IConfig;
import MAS.simulation.util.Settings;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.MasConfiguration;
import MAS.util.Tick;

class Stochastics {
	private Random random;
	private IConfig config;
	private double meanProcessingTime;

	Stochastics(IConfig config) {
		random = new Random();
		this.config = config;

		int countCapability = 0;
		double sumCapability = 0;
		Map<String, Pair<Position, List<Capability>>> equipletsConfigurations = config.getEquipletsConfigurations();
		for (Entry<String, Pair<Position, List<Capability>>> entry : equipletsConfigurations.entrySet()) {
			for (Capability capability : entry.getValue().second) {
				sumCapability += capability.getDuration().doubleValue();
				countCapability++;
			}
		}

		// calculate the average production time for all equiplet for product interarrival time generation
		meanProcessingTime = sumCapability / countCapability;
	}

	/**
	 * @return the mean processing time of equiplets
	 */
	public double getMeanProcessingTime() {
		return meanProcessingTime;
	}

	/**
	 * Generate the time a new product will arrive
	 * 
	 * @param current
	 *            time
	 * @return interarrival time of product
	 */
	public Tick generateProductArrival(Tick time) {
	if (time.greaterThan(Settings.WARMUP)) {
			int equiplets = config.getEquipletsConfigurations().size();

			double interarrival = (meanProcessingTime * Settings.MEAN_PRODUCT_STEPS) / (Settings.UTILIZATION * equiplets);
			return new Tick(interarrival);
		} else {
			return new Tick(10);
		}
	}

	/**
	 * Generate product deadline
	 * The distribution correspond to the given distribution in the simulation configurations
	 * 
	 * @return product deadline
	 */
	public Tick generateDeadline() {
		return time(config.getProductDeadline());
	}

	/**
	 * Generate the product steps for a new product
	 * 
	 * @return a list of product steps
	 */
	public LinkedList<ProductStep> generateProductSteps() {
		if (Settings.CONSTANT_NUMBER_OF_PRODUCT_STEPS) {
			return generateProductStepsRandom();
		} else if (!Settings.CONSTANT_NUMBER_OF_PRODUCT_STEPS) {
			return generateProductStepsUniformAmount();
		} else {
			return generateProductStepsTest();
		}
	}

	/**
	 * Generate the product steps with MEAN_PRODUCT_STEPS steps
	 * 
	 * @return a list of product steps
	 */
	private LinkedList<ProductStep> generateProductStepsRandom() {
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();
		
		for (int i = 0; i < Settings.MEAN_PRODUCT_STEPS ; i++) {
			int x = random.nextInt(productSteps.size());
			steps.add(new ProductStep(i, productSteps.get(x).getService(), productSteps.get(x).getCriteria()));
		}
		return steps;
	}

	/**
	 * Generate the product steps
	 * The number of product steps has a uniform distribution with a mean of MEAN_PRODUCT_STEPS and a minimum of MIN_PRODUCT_STEPS
	 * 
	 * @return a list of product steps
	 */
	private LinkedList<ProductStep> generateProductStepsUniformAmount() {
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();

		int b = 2 * Settings.MEAN_PRODUCT_STEPS - Settings.MIN_PRODUCT_STEPS;
		double ps = uniform(Settings.MIN_PRODUCT_STEPS, b);

		for (int i = 0; i < ps; i++) {
			int x = random.nextInt(productSteps.size());
			steps.add(new ProductStep(i, productSteps.get(x).getService(), productSteps.get(x).getCriteria()));
		}
		return steps;
	}

	/**
	 * Generate the product steps
	 * Test function
	 * 
	 * @return a list of product steps
	 */
	private LinkedList<ProductStep> generateProductStepsTest() {
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();
		int minProductSteps = 3;
		int maxProductSteps = 6;
		int n = minProductSteps + random.nextInt(maxProductSteps - minProductSteps);
		for (int i = 0; i < n; i++) {
			double u = random.nextDouble() * 100;
			int sum = 0;
			for (ProductStep productStep : productSteps) {
				sum += config.getProductStepProbablity(productStep);
				if (u <= sum) {
					steps.add(new ProductStep(i, productStep.getService(), productStep.getCriteria()));
					break;
				}
			}
		}
		return steps;
	}

	/**
	 * Generate the travel time for a number of squares in the grid
	 * 
	 * @param travelSquares
	 * @return travel time
	 */
	public Tick generateTravelTime(int travelSquares) {
		return time(config.getTravelTime()).multiply(travelSquares);
	}

	/**
	 * Generate the production time of a job
	 * 
	 * @param equiplet
	 *            performing the job
	 * @param service
	 *            of the job
	 * @return production time
	 */
	public Tick generateProductionTime(String equiplet, String service) {
		return time(config.equipletProductionTime(equiplet, service));
	}

	/**
	 * Generate the breakdown time
	 * 
	 * @param equiplet
	 *            that will breakdown
	 * @return time between the next breakdown
	 */
	public Tick generateBreakdownTime(String equiplet) {
		return time(config.equipletBreakdownTime(equiplet));
	}

	public Tick generateRepairTime(String equiplet) {
		return time(config.equipletRepaireTime(equiplet));
	}

	/**
	 * Generate the time it takes to reconfigure an equiplet
	 * 
	 * @return
	 */
	public Tick generateReconfigTime() {
		return new Tick(Settings.RECONFIGATION_TIME);
	}

	private Tick time(Pair<Tick, DurationType> data) {
		return time(data.first, data.second);
	}

	private Tick time(Tick time, DurationType type) {
		if (!Settings.STOCHASTICS) {
			return time;
		}
		switch (type) {
		case EXP:
			return new Tick(exp(time.doubleValue()));
		case WEIBULL:
			// return weibull(1, time);
		case GAMMA:
			// return gamma(1, time);
		case NORMAL:
		case DETERMINISTIC:
		default:
			return time;
		}
	}

	/**
	 * 
	 * @param mean
	 * @return a random variable with exponential distribution
	 */
	private double exp(double mean) {
		double u = random.nextDouble();
		return -mean * Math.log(1 - u);
	}

	/**
	 * 
	 * @param a
	 * @param b
	 * @return random variable with a gamma distribution
	 */
	protected double gamma(double a, double b) {
		// assert( b > 0. && c > 0. );
		double A = 1.0 / Math.sqrt(2 * b - 1);
		double B = b - Math.log(4);
		double Q = b + 1 / A;
		double T = 4.5;
		double D = 1 + Math.log(T);
		double C = 1 + b / Math.E;

		if (b < 1.0) {
			while (true) {
				double u = random.nextDouble();
				double p = C * u;

				if (p > 1) {
					double y = -Math.log((C - p) / b);
					double u2 = random.nextDouble();
					if (u2 <= Math.pow(y, b - 1)) {
						return a * y;
					}
				} else {
					double y = Math.pow(p, 1 / b);
					double u2 = random.nextDouble();
					if (u2 <= Math.exp(-y)) {
						return a * y;
					}
				}
			}
		} else if (b == 1.0) {
			return exp(a);
		} else {
			while (true) {
				double u1 = random.nextDouble();
				double u2 = random.nextDouble();
				double v = A * Math.log(u1 / (1 - u1));
				double y = b * Math.exp(v);
				double z = u1 * u1 * u2;
				double w = B + Q * v - y;
				if (w + D - T * z >= 0 || w >= Math.log(z)) {
					return a * y;
				}
			}
		}
	}

	protected double weibull(double A, double B) {
		// F−1(u) = [−αln(1−u)]1/β 0 < u < 1.
		double u = random.nextDouble();
		return Math.pow(-A * Math.log(1 - u), 1 / B);
	}

	/**
	 * mean = 1/2 ( a + b )
	 * 
	 * @param a
	 *            minimal value
	 * @param b
	 *            maximal value
	 * @return a random variable with a uniform distribution
	 */
	protected double uniform(double a, double b) {
		double u = random.nextDouble();
		return a + u * (b - a);
	}
}
