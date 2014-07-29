package simulation.simulation;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import simulation.config.Config;
import simulation.config.DurationType;
import simulation.mas.product.ProductStep;
import simulation.util.Pair;

class Stochastics {
	private Random random;
	private Config config;

	Stochastics(Config config) {
		random = new Random();
		this.config = config;
	}

	public double generateProductArrival() {
		return 35; // TODO too deterministic
		// return time(config.getProductArrival());
	}

	public double generateDeadline() {
		// TODO Auto-generated method stub
		return 0;
	}

	public double generateDuetime() {
		// TODO Auto-generated method stub
		return 0;
	}

	public LinkedList<ProductStep> generateProductSteps() {
		if (true) {
			 return generateProductStepsTest();
		}
		
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();
		int n = 2 + (int) (random.nextDouble() * (productSteps.size() - 2));
		for (int i = 0; i < n; i++) {
			double u = random.nextDouble() * 100;
			int sum = 0;
			for (ProductStep productStep : productSteps) {
				sum += config.getProductStepProbablity(productStep);
				if (u <= sum) {
					steps.add(productStep);
					break;
				}
			}
		}
		return steps;
	}

	private LinkedList<ProductStep> generateProductStepsTest() {
		LinkedList<ProductStep> steps = new LinkedList<>();
		List<ProductStep> productSteps = config.getProductSteps();
		
		steps.add(productSteps.get(0));
		steps.add(productSteps.get(1));
		steps.add(productSteps.get(2));
		steps.add(productSteps.get(3));
	
		return steps;
	}

	public double generateTravelTime(int travelSquares) {
		return travelSquares * time(config.getTravelTime());
	}

	public double generateProductionTime(String equiplet, String service) {
		return time(config.equipletProductionTime(equiplet, service));
	}

	public double generateBreakdownTime(String equiplet) {
		return 100;
		//return time(config.equipletBreakdownTime(equiplet));
	}

	public double generateRepairTime(String equiplet) {
		return time(config.equipletRepaireTime(equiplet));
	}

	private double time(Pair<Double, DurationType> data) {
		return time(data.first, data.second);
	}

	private double time(double time, DurationType type) {
		switch (type) {
		case EXP:
			//return exp(time);
		case WEIBULL:
			//return weibull(1, time);
		case GAMMA:
			//return gamma(1, time);
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
		// F−1(u) = [−αln(1−u)]1/β      0 < u < 1.
		double u = random.nextDouble();
		return Math.pow(-A * Math.log(1 - u), 1 / B);
	}
}
