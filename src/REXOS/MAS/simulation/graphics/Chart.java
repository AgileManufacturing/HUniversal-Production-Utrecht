package MAS.simulation.graphics;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.LegendItem;
import org.jfree.chart.LegendItemCollection;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import MAS.util.Tick;

public class Chart {

	private JFrame frame;
	private JFreeChart chart;

	public Chart(String title, String yLabel, Map<String, Map<Double, Double>> data) {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		for (Entry<String, Map<Double, Double>> entry : data.entrySet()) {
			final XYSeries series = new XYSeries(entry.getKey());
			for (Entry<Double, Double> point : entry.getValue().entrySet()) {
				series.add(point.getKey(), point.getValue());
			}
			dataset.addSeries(series);
		}

		chart = createChart(title, yLabel, dataset);

		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setMouseWheelEnabled(true);

		frame = new JFrame(title);
		frame.setTitle(title);
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setLayout(new BorderLayout(0, 5));
		frame.add(chartPanel, BorderLayout.CENTER);

		JPanel panel = new JPanel(new FlowLayout(FlowLayout.RIGHT));
		panel.setBackground(Color.WHITE);
		panel.add(createCloseButton());
		frame.add(panel, BorderLayout.SOUTH);
		frame.pack();
		frame.setSize(new Dimension(720, 500));
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}

	private JButton createCloseButton() {
		final JButton auto = new JButton(new AbstractAction("Close") {
			private static final long serialVersionUID = 1L;

			@Override
			public void actionPerformed(ActionEvent e) {
				frame.dispose();
			}
		});
		return auto;
	}

	private static JFreeChart createChart(String title, String yLabel, XYDataset dataset) {
		// create the chart...
		final JFreeChart chart = ChartFactory.createXYLineChart(title, // chart title
				"Time", // x axis label
				yLabel, // y axis label
				dataset, // data
				PlotOrientation.VERTICAL, true, // include legend
				true, // tooltips
				false // urls
		);

		// Optional customisation of chart
		chart.setBackgroundPaint(Color.WHITE);

		final XYPlot plot = chart.getXYPlot();
		plot.setBackgroundPaint(Color.WHITE);
		plot.setDomainGridlinePaint(Color.WHITE);
		plot.setRangeGridlinePaint(Color.LIGHT_GRAY);

		plot.getRenderer().setBaseStroke(new BasicStroke(1.5f));

		// for (int i = 0; i < paramCount - 1; i++) // for each time series

		/*
		 * final XYLineAndShapeRenderer renderer = (XYLineAndShapeRenderer) plot.getRenderer();
		 * renderer.setBaseLegendShape(new Rectangle(15, 15));
		 * renderer.setSeriesStroke(
		 * 0, new BasicStroke(
		 * 2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
		 * 1.0f, new float[] {10.0f, 6.0f}, 0.0f
		 * )
		 * );
		 * /
		 * XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer(){
		 * private static final long serialVersionUID = 1L;
		 * public Shape lookupLegendShape(int series) {
		 * return new Rectangle(15, 15);
		 * }
		 * };
		 */

		LegendItemCollection legendItems = plot.getLegendItems();
		for (int i = 0; i < legendItems.getItemCount(); i++) {
			LegendItem item = legendItems.get(i);
			item.setShape(new Rectangle(15, 15));
		}
		plot.setFixedLegendItems(legendItems);

		// renderer.setBaseLegendShape(new Rectangle(15, 15));
		// plot.setRenderer(renderer);

		return chart;
	}
	
	public static <M extends Map<Tick, T>, T extends Number> void save(String path, String title, String yLabel, Map<String, M> data) {
		try {
			File file = new File(path + title.replace(" ", "_") + ".png");

			final XYSeriesCollection dataset = new XYSeriesCollection();
			for (Entry<String, M> entry : data.entrySet()) {
				final XYSeries series = new XYSeries(entry.getKey());
				for (Entry<Tick, T> point : entry.getValue().entrySet()) {
					series.add(point.getKey().doubleValue(), point.getValue().doubleValue());
				}
				dataset.addSeries(series);
			}

			final JFreeChart chart = createChart(title, yLabel, dataset);
			ChartUtilities.saveChartAsPNG(file, chart, 1024, 720);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public static <M extends TreeMap<T, T>, T extends Number> void saves(String path, String title, String yLabel, Map<String, M> data, boolean fuckjava) {
		try {
			File file = new File(path + title.replace(" ", "_") + ".png");

			final XYSeriesCollection dataset = new XYSeriesCollection();
			for (Entry<String, M> entry : data.entrySet()) {
				final XYSeries series = new XYSeries(entry.getKey());
				for (Entry<T, T> point : entry.getValue().entrySet()) {
					series.add(point.getKey().doubleValue(), point.getValue().doubleValue());
				}
				dataset.addSeries(series);
			}

			final JFreeChart chart = createChart(title, yLabel, dataset);
			ChartUtilities.saveChartAsPNG(file, chart, 1024, 720);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 * @param title
	 *            of chart
	 * @param yLabel
	 *            label of y-axis
	 * @param data
	 *            <name [<name, start, end>]
	 * @return chart
	 */
	public static <M extends Map<Tick, T>, T extends Number> JPanel createChart(String title, String yLabel, Map<String, M> data) {
	// public static JPanel createChart(String title, String yLabel, Map<String, Map<Tick, Double>> data) {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		for (Entry<String, M> entry : data.entrySet()) {
			final XYSeries series = new XYSeries(entry.getKey());
			for (Entry<Tick, T> point : entry.getValue().entrySet()) {
				series.add(point.getKey().doubleValue(), point.getValue());
			}
			dataset.addSeries(series);
		}

		final JFreeChart chart = createChart(title, yLabel, dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		return chartPanel;
	}

	/**
	 * 
	 * @param title
	 * @param yLabel
	 * @param data1
	 * @param data
	 * @return
	 */
	public static <M extends Map<Tick, T>, T extends Number> Component createChart(String title, String yLabel1, String yLabel2, Map<String, M> stats, Map<String, Map<? extends Number, ? extends Number>> data) {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		for (Entry<String, M> entry : stats.entrySet()) {
			final XYSeries series = new XYSeries(entry.getKey());
			for (Entry<Tick, T> point : entry.getValue().entrySet()) {
				series.add(point.getKey().doubleValue(), point.getValue());
			}
			dataset.addSeries(series);
		}

		final JFreeChart chart = createChart(title, yLabel1, dataset);

		// second axis
		final XYSeriesCollection dataset2 = new XYSeriesCollection();
		for (Entry<String, Map<? extends Number, ? extends Number>> entry : data.entrySet()) {
			final XYSeries series = new XYSeries(entry.getKey());
			for (Entry<? extends Number, ? extends Number> point : entry.getValue().entrySet()) {
				series.add(point.getKey().doubleValue(), point.getValue());
			}
			dataset2.addSeries(series);
		}

		final XYPlot plot = chart.getXYPlot();
		final NumberAxis axis2 = new NumberAxis(yLabel2);

		plot.setRangeAxis(1, axis2);
		plot.setDataset(1, dataset2);
		plot.mapDatasetToRangeAxis(1, 1);

		LegendItemCollection legendItems = plot.getLegendItems();
		for (int i = 0; i < legendItems.getItemCount(); i++) {
			LegendItem item = legendItems.get(i);
			item.setShape(new Rectangle(15, 15));
		}
		plot.setFixedLegendItems(legendItems);

		final ChartPanel chartPanel = new ChartPanel(chart);
		return chartPanel;
	}

	public static JPanel createChartTicks(String title, String yLabel, Map<String, Map<Tick, Tick>> data) {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		for (Entry<String, Map<Tick, Tick>> entry : data.entrySet()) {
			final XYSeries series = new XYSeries(entry.getKey());
			for (Entry<Tick, Tick> point : entry.getValue().entrySet()) {
				series.add(point.getKey().doubleValue(), point.getValue().doubleValue());
			}
			dataset.addSeries(series);
		}

		final JFreeChart chart = createChart(title, yLabel, dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		return chartPanel;
	}

	/**
	 * Marker for current time
	 * Long timestampToMark = new Date().getTime();
	 * Marker m = new ValueMarker(timestampToMark);
	 * m.setStroke(new BasicStroke(2));
	 * m.setPaint(Color.RED);
	 * plot.addDomainMarker(m);
	 */
}
