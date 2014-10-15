package MAS.simulation.simulation;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import MAS.simulation.config.DurationType;
import MAS.simulation.config.IConfig;
import MAS.simulation.mas.product.ProductStep;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Settings;
import MAS.simulation.util.Tick;

class Stochastics {
	private Random random;
	private IConfig config;

	Stochastics(IConfig config) {
		random = new Random();
		this.config = config;
	}

	int x = 0;

	public Tick generateProductArrival() {
		return time(config.getProductArrival());
		/*
		 * if (x < 15) {
		 * x++;
		 * return 100;
		 * } else {
		 * return 10000000;
		 * }
		 */
		// return 10; // TODO too deterministic
		// return time(config.getProductArrival());
	}

	public Tick generateDeadline() {
		return time(config.getProductDeadline());
	}

	public LinkedList<ProductStep> generateProductSteps() {
		if (true) {
			return generateProductStepsTest();
			/*
			 * LinkedList<ProductStep> steps = new LinkedList<>();
			 * List<ProductStep> productSteps = config.getProductSteps();
			 * 
			 * int minProductSteps = 3;
			 * int avgProductSteps = productSteps.size();
			 * int n = minProductSteps + random.nextInt(avgProductSteps -
			 * minProductSteps);
			 * for (int i = 0; i < n; i++) {
			 * ProductStep step =
			 * productSteps.get(random.nextInt(productSteps.size()));
			 * steps.add(new ProductStep(i, step.getService(),
			 * step.getCriteria()));
			 * }
			 * 
			 * return steps;
			 */
		}

		@SuppressWarnings("unused")
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

	private LinkedList<ProductStep> generateProductStepsTest() {
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();

		for (int i = 0; i < productSteps.size() * 2; i++) {
			int x = i % productSteps.size(); // random.nextInt(productSteps.size());
			steps.add(new ProductStep(i, productSteps.get(x).getService(), productSteps.get(x).getCriteria()));
		}
		return steps;
	}

	public Tick generateTravelTime(int travelSquares) {
		return time(config.getTravelTime()).times(travelSquares);
	}

	public Tick generateProductionTime(String equiplet, String service) {
		return time(config.equipletProductionTime(equiplet, service));
	}

	public Tick generateBreakdownTime(String equiplet) {
		return time(config.equipletBreakdownTime(equiplet));
	}

	public Tick generateRepairTime(String equiplet) {
		return time(config.equipletRepaireTime(equiplet));
	}

	private Tick time(Pair<Tick, DurationType> data) {
		return time(data.first, data.second);
	}

	public Tick generateReconfigTime() {
		return new Tick(Settings.RECONFIGATION_TIME);
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

	private double exp(double mean) {
		double u = random.nextDouble();
		return -mean * Math.log(1 - u);
	}

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
}
