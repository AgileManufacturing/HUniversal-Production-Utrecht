package simulation.graphics;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.data.time.MovingAverage;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import simulation.util.Pair;
import simulation.util.Triple;

public class Chart {

	private JFrame frame;
	private JFreeChart chart;

	public Chart(String title, HashMap<Integer, Triple<Double, Double, ArrayList<Double>>> data) {
		Pair<XYDataset, XYDataset> dataset = createDataset(data);
		chart = createChart(dataset, title);

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

	public static Pair<XYDataset, XYDataset> createDataset(HashMap<Integer, Triple<Double, Double, ArrayList<Double>>> data) {
		final XYSeries series1 = new XYSeries("Denisty");
		//final XYSeries series2 = new XYSeries("Average Speed");
		ArrayList<XYSeries> speedSerie = new ArrayList<>();

		for (Entry<Integer, Triple<Double, Double, ArrayList<Double>>> entry : data.entrySet()) {
			series1.add(entry.getKey(), entry.getValue().first);
			//series2.add(entry.getKey(), entry.getValue().second);

			ArrayList<Double> speeds = entry.getValue().third;
			for (int i = 0; i < speeds.size(); i++) {
				if (speedSerie.size() <= i) {
					XYSeries s = new XYSeries("Speed lane " + i);
					s.add(entry.getKey(), speeds.get(i));
					speedSerie.add(s);
				} else {
					XYSeries s = speedSerie.get(i);
					s.add(entry.getKey(), speeds.get(i));
				}
			}
		}

		final XYSeriesCollection dataset1 = new XYSeriesCollection();
		final XYSeriesCollection dataset2 = new XYSeriesCollection();
		dataset1.addSeries(series1);
		//dataset2.addSeries(series2);

		for (int i = 0; i < speedSerie.size(); i++) {
			// for (XYSeries xySeries : speedSerie) {
			dataset2.addSeries(speedSerie.get(i));
			dataset2.addSeries(MovingAverage.createMovingAverage(dataset2, dataset2.getSeriesCount() - 1, "Velocity lane " + (i + 1), data.size() * 0.01, data.size() * 0.005));
			dataset2.removeSeries(speedSerie.get(i));
		}

		dataset1.addSeries(MovingAverage.createMovingAverage(dataset1, 0, "Density", data.size() * 0.01, data.size() * 0.005));
		dataset1.removeSeries(series1);

		return new Pair<XYDataset, XYDataset>(dataset1, dataset2);
	}

	private static JFreeChart createChart(Pair<XYDataset, XYDataset> dataset, String title) {
		// create the chart...
		final JFreeChart chart = ChartFactory.createXYLineChart(title, // chart title
				"Time", // x axis label
				"Vehicle Speed", // y axis label
				dataset.second, // data
				PlotOrientation.VERTICAL, true, // include legend
				true, // tooltips
				false // urls
		);

		// Optional customisation of chart
		chart.setBackgroundPaint(Color.white);

		//final StandardLegend legend = (StandardLegend) chart.getLegend();
		// legend.setDisplaySeriesShapes(true);

		// get a reference to the plot for further customisation...
		final XYPlot plot = chart.getXYPlot();
		plot.setBackgroundPaint(Color.lightGray);
		// plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
		plot.setDomainGridlinePaint(Color.white);
		plot.setRangeGridlinePaint(Color.white);

		// TODO uncomment this 
//		ValueAxis axis = plot.getRangeAxis();
//		axis.setRange(0, 25.5);
//
//		final StandardXYItemRenderer renderer = new StandardXYItemRenderer();
//		// renderer.setSeriesLinesVisible(0, false);
//		// renderer.setSeriesShapesVisible(1, false);
//		plot.setRenderer(0, renderer);
//
//
//		// change the auto tick unit selection to integer units only...
//		final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
//		rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
//
//		final NumberAxis axis2 = new NumberAxis("Density");
//		axis2.setAutoRangeIncludesZero(false);
//		axis2.setRange(0.0, 0.11);
//		plot.setRangeAxis(1, axis2);
		plot.setDataset(1, dataset.first);
		plot.mapDatasetToRangeAxis(1, 1);

		final StandardXYItemRenderer renderer2 = new StandardXYItemRenderer();
		renderer2.setSeriesPaint(0, Color.black);
		plot.setRenderer(1, renderer2);

		return chart;
	}

	public static void save(String path, HashMap<Integer, Triple<Double, Double, ArrayList<Double>>> data) {
		try {
			File file = new File(path + "statistics.png");
			Pair<XYDataset, XYDataset> dataset = createDataset(data);
			JFreeChart chart = createChart(dataset, "");
			ChartUtilities.saveChartAsPNG(file, chart, 1024, 720);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
