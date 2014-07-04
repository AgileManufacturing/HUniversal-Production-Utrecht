package simulation.graphics;

import java.awt.Color;
import java.awt.Dimension;
import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.category.DefaultCategoryDataset;
import org.jfree.ui.ApplicationFrame;

import simulation.util.Triple;

public class StackedBarChart extends ApplicationFrame {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public StackedBarChart(String title, Map<String, Triple<Double, Double, Double>> data) {
		super(title);

		final DefaultCategoryDataset dataset = createDataset(data);
		final JFreeChart chart = createChart(title, dataset);

		// add the chart to a panel...
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(800, 400));
		setContentPane(chartPanel);
	}

	public static JPanel createChartPanel(Map<String, Triple<Double, Double, Double>> data) {
		final DefaultCategoryDataset dataset = createDataset(data);
		final JFreeChart chart = createChart("Schedule", dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		// chartPanel.setPreferredSize(new Dimension(800, 400));
		return chartPanel;
	}

	/**
	 * Creates a sample dataset for a Gantt chart.
	 * 
	 * @return The dataset.
	 */
	private static DefaultCategoryDataset createDataset(Map<String, Triple<Double, Double, Double>> data) {
		DefaultCategoryDataset collection = new DefaultCategoryDataset();
		for (Entry<String, Triple<Double, Double, Double>> entry : data.entrySet()) {
			collection.addValue(entry.getValue().first, "idle", entry.getKey());
			collection.addValue(entry.getValue().second, "busy", entry.getKey());
			collection.addValue(entry.getValue().third, "broken", entry.getKey());
		}
		return collection;
	}

	/**
	 * Creates a chart.
	 * 
	 * @param dataset
	 *            the dataset.
	 * 
	 * @return The chart.
	 */
	private static JFreeChart createChart(final String title, final DefaultCategoryDataset dataset) {
		final JFreeChart chart = ChartFactory.createStackedBarChart(title, // chart title
				"Equiplets", // x axis label
				"Time", // y axis label
				dataset, // data
				PlotOrientation.VERTICAL, true, // include legend
				true, // tooltips
				false // urls
		);

		// Optional customisation of chart
		chart.setBackgroundPaint(Color.WHITE);

		CategoryPlot plot = chart.getCategoryPlot();
		plot.setBackgroundPaint(Color.WHITE);
		plot.setRangeGridlinePaint(Color.LIGHT_GRAY);
		
		return chart;
	}

	public static void save(String filename, String title, Map<String, Triple<Double, Double, Double>> data) {
		try {
			File file = new File(filename);
			final DefaultCategoryDataset dataset = createDataset(data);
			final JFreeChart chart = createChart(title, dataset);
			ChartUtilities.saveChartAsPNG(file, chart, 1024, 720);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
