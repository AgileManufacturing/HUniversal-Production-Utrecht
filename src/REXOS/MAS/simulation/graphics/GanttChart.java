package simulation.graphics;

import java.awt.Color;
import java.awt.Dimension;
import java.text.DateFormat;
import java.text.FieldPosition;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.data.category.IntervalCategoryDataset;
import org.jfree.data.gantt.Task;
import org.jfree.data.gantt.TaskSeries;
import org.jfree.data.gantt.TaskSeriesCollection;
import org.jfree.data.time.SimpleTimePeriod;
import org.jfree.ui.ApplicationFrame;

import simulation.mas.Product;
import simulation.util.ProductionStep;

public class GanttChart extends ApplicationFrame {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public GanttChart(String title, ArrayList<TaskSeries> tasks) {
		super(title);

		final IntervalCategoryDataset dataset = createDataset(tasks);
		final JFreeChart chart = createChart(title, dataset);

		// add the chart to a panel...
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(800, 400));
		setContentPane(chartPanel);
	}

	/**
	 * Creates a sample dataset for a Gantt chart.
	 * 
	 * @return The dataset.
	 */
	private static IntervalCategoryDataset createDataset(ArrayList<TaskSeries> tasks) {
		final TaskSeriesCollection collection = new TaskSeriesCollection();
		for (TaskSeries task : tasks) {
			collection.add(task);
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
	private static JFreeChart createChart(final String title, final IntervalCategoryDataset dataset) {

		final JFreeChart chart = ChartFactory.createGanttChart(title, // chart title
				"Equiplets", // x axis label
				"Time", // y axis label
				dataset, // data
				true, // include legend
				true, // tooltips
				false // urls
		);

		// Optional customisation of chart
		chart.setBackgroundPaint(Color.white);

		CategoryPlot plot = chart.getCategoryPlot();
		plot.setBackgroundPaint(Color.WHITE);
		plot.setRangeAxisLocation(AxisLocation.BOTTOM_OR_RIGHT);

		DateAxis axis = (DateAxis) plot.getRangeAxis();
		axis.setDateFormatOverride(new DateFormat() {
			private static final long serialVersionUID = 1L;

			@Override
			public Date parse(String source, ParsePosition pos) {
				Calendar calendar = Calendar.getInstance();
				try {
					long time = Long.parseLong(source);
					calendar.setTimeInMillis(time);
					return calendar.getTime();
				} catch (NumberFormatException e) {
					return new Date();
				}
			}

			@Override
			public StringBuffer format(Date date, StringBuffer toAppendTo, FieldPosition fieldPosition) {
				return toAppendTo.append(String.valueOf(date.getTime()));
			}
		});

		return chart;
	}

	public static JPanel createChartPanel(List<Product> agents) {
		ArrayList<TaskSeries> tasks = new ArrayList<>();
		for (Product agent : agents) {
			LinkedList<ProductionStep> path = agent.getProductionPath();
			TaskSeries serie = new TaskSeries(agent.getName());
			for (ProductionStep node : path) {
				serie.add(new Task(node.getEquiplet(), new SimpleTimePeriod((long) (node.getTime() * 1000), (long) ((node.getTime() + node.getDuration()) * 1000))));
			}
			tasks.add(serie);
		}

		final IntervalCategoryDataset dataset = createDataset(tasks);
		final JFreeChart chart = createChart("Schedule", dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		// chartPanel.setPreferredSize(new Dimension(800, 400));
		return chartPanel;
	}

}
