package MAS.simulation.test;

import org.junit.Test;

import MAS.simulation.util.Settings;

public class StochasticDistributions {

	@Test
	public void testUniform() {
		Stochastics stochastics = Stochastics.getInstance();
		int b = 2 * Settings.MEAN_PRODUCT_STEPS - Settings.MIN_PRODUCT_STEPS;
		System.out.println("Test: Uniform [mean=" + Settings.MEAN_PRODUCT_STEPS + ", a=" + Settings.MIN_PRODUCT_STEPS + ", b=" + b + "]");
		for (int i = 0; i < 100; i++) {
			double ps = stochastics.uniform(Settings.MIN_PRODUCT_STEPS, b);
			System.out.println(ps);
		}
	}

	@Test
	public void testExp() {
		Stochastics stochastics = Stochastics.getInstance();
		System.out.println("Test: Exp [mean=" + Settings.MEAN_PRODUCT_STEPS + "]");
		for (int i = 0; i < 100; i++) {
			double ps = stochastics.exp(Settings.MEAN_PRODUCT_STEPS);
			System.out.println(ps);
		}
	}

	@Test
	public void testGamma() {
		Stochastics stochastics = Stochastics.getInstance();
		double shape = 5.0;
		double scale = 0.5;
		System.out.println("Test: Gamma [shape=" + shape + ", scale=" + scale + "]");
		for (int i = 0; i < 100; i++) {
			double ps = stochastics.gamma(shape, scale);
			System.out.println(ps);
		}
	}
}
