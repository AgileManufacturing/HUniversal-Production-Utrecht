package MAS.simulation.test;

import java.util.Random;

class Stochastics {
	private Random random;
	private static Stochastics stochastics = null;

	private Stochastics() {
		random = new Random();
	}

	public static Stochastics getInstance() {
		if (stochastics == null) {
			stochastics = new Stochastics();
		}
		return stochastics;
	}

	protected double uniform(double a, double b) {
		double u = random.nextDouble();
		return a + u * (b - a);
	}
	
	double exp(double mean) {
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