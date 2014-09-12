package MAS.simulation.graphics;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.text.DateFormat;
import java.text.FieldPosition;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;

import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.CategoryAxis;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.renderer.category.GanttRenderer;
import org.jfree.data.category.IntervalCategoryDataset;
import org.jfree.data.gantt.Task;
import org.jfree.data.gantt.TaskSeries;
import org.jfree.data.gantt.TaskSeriesCollection;
import org.jfree.data.time.SimpleTimePeriod;
import org.jfree.ui.ApplicationFrame;

import MAS.simulation.util.Tick;
import MAS.simulation.util.Triple;

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

		CategoryAxis domainAxis = plot.getDomainAxis();
		domainAxis.setCategoryMargin(0.05);
		domainAxis.setLowerMargin(0.05);
		domainAxis.setUpperMargin(0.05);

		GanttRenderer renderer = (GanttRenderer) plot.getRenderer();
		renderer.setItemMargin(0);
		renderer.setDrawBarOutline(true);
		renderer.setBaseOutlineStroke(new BasicStroke(0));
		// renderer.set

		return chart;
	}

	/**
	 * 
	 * @param data
	 *            <name [<name, start, end>]
	 * @return chart
	 */
	public static JPanel createChart(String title, String yLabel, Map<String, List<Triple<String, Tick, Tick>>> data) {
		int counter = 0;
		ArrayList<TaskSeries> tasks = new ArrayList<>();
		TreeSet<String> yLabels = new TreeSet<>();
		for (Entry<String, List<Triple<String, Tick, Tick>>> entry : data.entrySet()) {
			TaskSeries serie = new TaskSeries(entry.getKey());

			for (Triple<String, Tick, Tick> value : entry.getValue()) {
				yLabels.add(value.first);

				Task task = serie.get(value.first);
				if (task == null) {
					task = new Task(value.first, new SimpleTimePeriod(value.second.longValue(), value.third.longValue()));
					task.addSubtask(new Task(value.first, new SimpleTimePeriod(value.second.longValue(), value.third.longValue())));
					serie.add(task);
				} else {
					task.addSubtask(new Task(value.first, new SimpleTimePeriod(value.second.longValue(), value.third.longValue())));
				}
				counter++;
			}
			tasks.add(serie);
		}

		TaskSeriesCollection dataset = new TaskSeriesCollection();
		TaskSeries forceSortedLabels = new TaskSeries("");
		for (String yItem : yLabels) {
			forceSortedLabels.add(new Task(yItem, new SimpleTimePeriod(0, 0)));
			;
		}
		dataset.add(forceSortedLabels);

		for (TaskSeries serie : tasks) {
			dataset.add(serie);
		}

		final JFreeChart chart = createChart(title, yLabel, dataset);

		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setMaximumDrawHeight(Math.max(counter * 20, 800));
		chartPanel.setPreferredSize(new Dimension(800, Math.max(counter * 20, 800)));
		return chartPanel;
	}

	/**
	 * Create a gantt chart panel. The data is based on basis of y-axis labels with a list of x-axis data.
	 * For example, a Map < equiplet as keys, and a list < of start and end time for product representing jobs > >
	 * 
	 * @param title
	 *            of the chart
	 * @param yLabel
	 *            label of the y-axis
	 * @param data
	 *            <y-axis, [name, start, end] >
	 * @return
	 */
	public static JPanel createChartInvert(String title, String yLabel, Map<String, List<Triple<String, Tick, Tick>>> data) {
		Tick maxTime = new Tick(300);
		Tick minTime = new Tick(Double.MAX_VALUE);

		TaskSeriesCollection dataset = new TaskSeriesCollection();
		TreeMap<String, TaskSeries> tasks = new TreeMap<>();
		TreeSet<String> yLabels = new TreeSet<>();

		for (Entry<String, List<Triple<String, Tick, Tick>>> entry : data.entrySet()) {
			yLabels.add(entry.getKey());
			for (Triple<String, Tick, Tick> value : entry.getValue()) {

				if (!tasks.containsKey(value.first)) {
					tasks.put(value.first, new TaskSeries(value.first));
				}

				TaskSeries serie = tasks.get(value.first);
				Task task = serie.get(entry.getKey());
				if (task == null) {
					task = new Task(entry.getKey(), new SimpleTimePeriod(value.second.longValue(), value.third.longValue()));
					task.addSubtask(new Task(entry.getKey(), new SimpleTimePeriod(value.second.longValue(), value.third.longValue())));
					serie.add(task);
				} else {
					task.addSubtask(new Task(entry.getKey(), new SimpleTimePeriod(value.second.longValue(), value.third.longValue())));
				}
				maxTime = maxTime.max(value.third);
				minTime = minTime.min(value.second);
			}
		}

		System.out.println(" MIN:" + minTime + "  MAX:" + maxTime + " data:\n" + data);

		TaskSeries forceSortedLabels = new TaskSeries("");
		for (String yItem : yLabels) {
			forceSortedLabels.add(new Task(yItem, new SimpleTimePeriod(minTime.longValue(), minTime.longValue())));
		}
		dataset.add(forceSortedLabels);

		for (TaskSeries serie : tasks.values()) {
			dataset.add(serie);
		}

		final JFreeChart chart = createChart(title, yLabel, dataset);

		CategoryPlot plot = chart.getCategoryPlot();
		ValueAxis axis = plot.getRangeAxis();
		axis.setLowerBound(0);
		axis.setRange((minTime.greaterThan(100) && minTime.lessThan(Double.MAX_VALUE) ? minTime.longValue() - 100 : 0), maxTime.longValue() + 100);

		final ChartPanel chartPanel = new ChartPanel(chart);

		chartPanel.setMaximumDrawWidth(Math.max(maxTime.intValue() - minTime.intValue(), 800));
		chartPanel.setPreferredSize(new Dimension(Math.max(maxTime.intValue() - minTime.intValue(), 800), 800));
		return chartPanel;
	}
}
