package simulation.graphics;

import java.awt.Color;
import java.awt.Dimension;
import java.text.DateFormat;
import java.text.FieldPosition;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

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

import simulation.mas.Equiplet;
import simulation.mas.Job;
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
		final JFreeChart chart = createChart(title, title, dataset);

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
	private static IntervalCategoryDataset createDataset(List<TaskSeries> tasks) {
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
	private static JFreeChart createChart(final String title, final String yLabel, final IntervalCategoryDataset dataset) {

		final JFreeChart chart = ChartFactory.createGanttChart(title, // chart title
				yLabel, // x axis label
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
		axis.setMinimumDate(new Date(0));

		return chart;
	}


	public static JPanel createChartProducts(List<Product> agents) {
		double maxTime = 300;
		double minTime = Double.MAX_VALUE;
		ArrayList<TaskSeries> tasks = new ArrayList<>();
		for (Product agent : agents) {
			LinkedList<ProductionStep> path = agent.getProductionPath();
			TaskSeries serie = new TaskSeries(agent.getName());
			for (ProductionStep node : path) {
				serie.add(new Task(node.getEquiplet(), new SimpleTimePeriod((long) node.getTime(), (long) (node.getTime() + node.getDuration()))));
				maxTime = Math.max(maxTime, node.getTime() + node.getDuration());
				minTime = Math.min(minTime, node.getTime());
			}
			tasks.add(serie);
		}

		final IntervalCategoryDataset dataset = createDataset(tasks);
		final JFreeChart chart = createChart("Schedule", "Equiplets", dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setMaximumDrawWidth((int) (maxTime - minTime));
		chartPanel.setPreferredSize(new Dimension((int) (maxTime - minTime), 600));
		return chartPanel;
	}
	
	public static JPanel createChartEquiplets(List<Equiplet> equiplets) {
		double maxTime = 0;
		HashMap<String, ArrayList<Task>> products = new HashMap<String, ArrayList<Task>>();
		for (Equiplet equiplet : equiplets) {	
			List<Job> history = equiplet.getHistory();
			for (Job job : history) {
				if (!products.containsKey(job.getProductAgent())) {
					products.put(job.getProductAgent(), new ArrayList<Task>());
				}
				products.get(job.getProductAgent()).add(new Task(equiplet.getName(), new SimpleTimePeriod((long) job.getStartTime(), (long) job.getDueTime())));
				maxTime = Math.max(maxTime, job.getDueTime());
			}
		}

		ArrayList<TaskSeries> tasks = new ArrayList<>();
		for (Entry<String, ArrayList<Task>> product : products.entrySet()) {
			TaskSeries serie = new TaskSeries(product.getKey());
			for (Task task : product.getValue()) {
				serie.add(task);
			}
			tasks.add(serie);
		}
		
		final IntervalCategoryDataset dataset = createDataset(tasks);
		final JFreeChart chart = createChart("Schedule", "Equiplets", dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setMaximumDrawWidth((int) maxTime); 
		chartPanel.setPreferredSize(new Dimension((int) maxTime, 600));
		return chartPanel;
	}
	
	public static JPanel createChartEquiplets(List<Equiplet> equiplets, boolean productBased) {
		int productCount = 0;
		ArrayList<TaskSeries> tasks = new ArrayList<>();
		for (Equiplet equiplet : equiplets) {
			
			List<Job> history = equiplet.getHistory();
			TaskSeries serie = new TaskSeries(equiplet.getName());
			for (Job job : history) {
				serie.add(new Task(job.getProductAgent(), new SimpleTimePeriod((long)job.getStartTime(), (long)job.getDueTime())));
				productCount++;
			}
			tasks.add(serie);
		}
		
		final IntervalCategoryDataset dataset = createDataset(tasks);
		final JFreeChart chart = createChart("Schedule", "Products", dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setMaximumDrawHeight(productCount * 10);  
		chartPanel.setPreferredSize(new Dimension(600, productCount * 10));
		return chartPanel;
	}
}
