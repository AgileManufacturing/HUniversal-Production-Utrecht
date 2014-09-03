package simulation.graphics;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.EventQueue;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSeparator;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import simulation.mas.equiplet.Equiplet;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public class SimInterface {

	private JFrame frmRexosSimulation;
	private JLabel lblTime;
	private GridView gridView;
	private JPanel chartPanel;

	private Control simulation;
	private JLabel lblWaitingTime;
	private JLabel lblBusy;
	private JLabel lblTraveling;
	private JLabel lblProducts;
	private JLabel lblThroughput;
	private JButton btnStart;
	private JLabel lblEvent;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					// SimInterface window = new SimInterface();
					// Control simulation = new Simulation(window);
					// window.setSimulation(simulation);
					// window.initContent();
					System.out.println("not supported");
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public SimInterface() {

	}

	public static SimInterface create(final Control control) {
		SimInterface simInterface = new SimInterface();
		simInterface.setSimulation(control);
		simInterface.initContent();
		return simInterface;
	}

	protected void setSimulation(Control simulation) {
		this.simulation = simulation;
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initContent() {
		frmRexosSimulation = new JFrame();
		frmRexosSimulation.setTitle("REXOS Simulation");
		frmRexosSimulation.setBounds(100, 100, 1200, 700);
		frmRexosSimulation.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		final JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		frmRexosSimulation.getContentPane().add(tabbedPane, BorderLayout.CENTER);

		gridView = new GridView();
		gridView.setBackground(Color.WHITE);
		tabbedPane.addTab("Grid", null, gridView, null);
		tabbedPane.setFocusable(false);

		final JPanel equipletUtilizationView = new JPanel();
		tabbedPane.addTab("Equiplet Utilization", null, equipletUtilizationView, null);
		equipletUtilizationView.setLayout(new BoxLayout(equipletUtilizationView, BoxLayout.X_AXIS));

		final JScrollPane scrollPane = new JScrollPane();
		tabbedPane.addTab("Equiplet Schedule", null, scrollPane, null);

		chartPanel = new JPanel();
		scrollPane.setViewportView(chartPanel);
		chartPanel.setLayout(new BoxLayout(chartPanel, BoxLayout.X_AXIS));

		final JPanel equipletSchedule = new JPanel();
		equipletSchedule.setLayout(new BoxLayout(equipletSchedule, BoxLayout.Y_AXIS));

		final JScrollPane eScheduleScroll = new JScrollPane();
		eScheduleScroll.setViewportView(equipletSchedule);
		tabbedPane.addTab("Equiplet history", null, eScheduleScroll, null);

		final JPanel equipletLatency = new JPanel();
		equipletLatency.setLayout(new BoxLayout(equipletLatency, BoxLayout.Y_AXIS));

		final JScrollPane equipletLatencyScroll = new JScrollPane();
		equipletLatencyScroll.setViewportView(equipletLatency);
		tabbedPane.addTab("Equiplet Latency", null, equipletLatencyScroll, null);

		final JPanel productSchedule = new JPanel();
		productSchedule.setLayout(new BoxLayout(productSchedule, BoxLayout.Y_AXIS));

		final JScrollPane pScheduleScroll = new JScrollPane();
		pScheduleScroll.setViewportView(productSchedule);
		tabbedPane.addTab("Product schedules", null, pScheduleScroll, null);

		final ProductView productView = new ProductView();
		tabbedPane.addTab("Products", null, productView, null);

		final JPanel productStatistics = new JPanel();
		productStatistics.setLayout(new BoxLayout(productStatistics, BoxLayout.Y_AXIS));

		final JScrollPane productStatisticsScroll = new JScrollPane();
		productStatisticsScroll.setViewportView(productStatistics);
		tabbedPane.addTab("Product Statistics", null, productStatisticsScroll, null);
		
		final JPanel equipletStatistics = new JPanel();
		equipletStatistics.setLayout(new BoxLayout(equipletStatistics, BoxLayout.Y_AXIS));

		final JScrollPane equipletStatisticsScroll = new JScrollPane();
		equipletStatisticsScroll.setViewportView(equipletStatistics);
		tabbedPane.addTab("Equiplet Statistics", null, equipletStatisticsScroll, null);

		ChangeListener changeListener = new ChangeListener() {
			public void stateChanged(ChangeEvent changeEvent) {
				JTabbedPane sourceTabbedPane = (JTabbedPane) changeEvent.getSource();
				int index = sourceTabbedPane.getSelectedIndex();
				if (index == tabbedPane.indexOfComponent(equipletUtilizationView)) {
					// Equiplet Utilization
					Map<String, Triple<Double, Double, Double>> history = simulation.getEquipletUtilization();
					System.out.println("HISTORY: " + history);
					equipletUtilizationView.removeAll();
					equipletUtilizationView.add(StackedBarChart.createChartPanel(history));
				} else if (index == tabbedPane.indexOfComponent(scrollPane)) {
					// Equiplet Schedule

					// Map<String, Triple<Double, Double, Double>> history = simulation.getEquipletHistory();
					// System.out.println("HISTORY: " + history);

					Map<String, List<Triple<String, Double, Double>>> schedules = simulation.getEquipletSchedule();
					chartPanel.removeAll();
					chartPanel.add(GanttChart.createChartInvert("Equiplet Schedule", "Equiplets", schedules));

					// Map<String, Product> products = simulation.getProducts();
					// if (products != null) {
					// List<Product> agents = new ArrayList<Product>(products.values());
					// chartPanel.add(GanttChart.createChartProducts(agents));
					// }
				} else if (index == tabbedPane.indexOfComponent(eScheduleScroll)) {
					// Equiplet history
					Map<String, List<Triple<String, Double, Double>>> history = simulation.getEquipletHistory();
					if (history != null) {
						equipletSchedule.removeAll();
						equipletSchedule.add(GanttChart.createChartInvert("Equiplet History", "Equiplets", history));
					}

					// List<Equiplet> equiplets = simulation.getEquiplets();
					// equipletSchedule.removeAll();
					// equipletSchedule.add(GanttChart.createChartEquiplets(equiplets));
				} else if (index == tabbedPane.indexOfComponent(equipletLatencyScroll)) {
					// Equiplet Latency 
					Map<String, Map<Double, Double>> latency = simulation.getEquipletLatency();
					equipletLatency.removeAll();
					equipletLatency.add(Chart.createChart("Equiplet Latency", "Latency", latency));

				} else if (index == tabbedPane.indexOfComponent(pScheduleScroll)) {
					// Product schedules
					//					List<Equiplet> equiplets = simulation.getEquiplets();
					//					productSchedule.removeAll();
					//					productSchedule.add(GanttChart.createChartEquiplets(equiplets, true));

					Map<String, List<Triple<String, Double, Double>>> schedules = simulation.getCompleteSchedule();
					productSchedule.removeAll();
					productSchedule.add(GanttChart.createChart("Product Schedules", "Products", schedules));

				} else if (index == tabbedPane.indexOfComponent(productView)) {
					// Products
					//Map<String, Product> products = simulation.getProducts();
					//if (products != null) {
					//	productView.update(products);
					//}
				} else if (index == tabbedPane.indexOfComponent(productStatisticsScroll)) {
					// Product Statistics
					Map<String, Map<Double, Double>> stats = simulation.getProductStatistics();
					productStatistics.removeAll();
					productStatistics.add(Chart.createChart("Product Statistics", "Products", stats));

				}else if (index == tabbedPane.indexOfComponent(equipletStatisticsScroll)) {
					// Equiplet Statistics
					Map<String, Map<Double, Double>> stats = simulation.getEquipletStatistics();
					equipletStatistics.removeAll();
					equipletStatistics.add(Chart.createChart("Equiplet Statistics", "Equiplets", stats));

				}
				System.out.println("Tab changed to: " + sourceTabbedPane.getTitleAt(index));
			}
		};
		tabbedPane.addChangeListener(changeListener);

		JPanel optionsPanel = new JPanel();
		frmRexosSimulation.getContentPane().add(optionsPanel, BorderLayout.EAST);
		GridBagLayout gbl_optionsPanel = new GridBagLayout();
		gbl_optionsPanel.columnWidths = new int[] { 70, 0 };
		gbl_optionsPanel.rowHeights = new int[] { 15, 25, 0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 462, 0, 0, 0, 0, 0, 0 };
		gbl_optionsPanel.columnWeights = new double[] { 1.0, Double.MIN_VALUE };
		gbl_optionsPanel.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		optionsPanel.setLayout(gbl_optionsPanel);

		btnStart = new JButton("Start");
		btnStart.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (btnStart.getText().matches("Start")) {
					btnStart.setText("Pause");
				} else {
					btnStart.setText("Start");
				}
				simulation.pause();
			}
		});

		JLabel lblOptions = new JLabel("Options");
		lblOptions.setHorizontalAlignment(SwingConstants.CENTER);
		GridBagConstraints gbc_lblOptions = new GridBagConstraints();
		gbc_lblOptions.anchor = GridBagConstraints.WEST;
		gbc_lblOptions.insets = new Insets(0, 0, 5, 0);
		gbc_lblOptions.gridx = 0;
		gbc_lblOptions.gridy = 0;
		optionsPanel.add(lblOptions, gbc_lblOptions);
		GridBagConstraints gbc_btnStart = new GridBagConstraints();
		gbc_btnStart.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnStart.insets = new Insets(0, 15, 5, 15);
		gbc_btnStart.gridx = 0;
		gbc_btnStart.gridy = 1;
		optionsPanel.add(btnStart, gbc_btnStart);

		JButton btnStep = new JButton("Step");
		btnStep.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simulation.step();
			}
		});
		GridBagConstraints gbc_btnStep = new GridBagConstraints();
		gbc_btnStep.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnStep.insets = new Insets(0, 15, 5, 15);
		gbc_btnStep.gridx = 0;
		gbc_btnStep.gridy = 2;
		optionsPanel.add(btnStep, gbc_btnStep);

		JSeparator separator_1 = new JSeparator();
		GridBagConstraints gbc_separator_1 = new GridBagConstraints();
		gbc_separator_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_separator_1.insets = new Insets(0, 0, 5, 0);
		gbc_separator_1.gridx = 0;
		gbc_separator_1.gridy = 3;
		optionsPanel.add(separator_1, gbc_separator_1);

		JLabel lblTimeText = new JLabel("Time");
		lblTimeText.setHorizontalAlignment(SwingConstants.CENTER);
		GridBagConstraints gbc_lblTimeText = new GridBagConstraints();
		gbc_lblTimeText.insets = new Insets(0, 0, 5, 0);
		gbc_lblTimeText.anchor = GridBagConstraints.WEST;
		gbc_lblTimeText.gridx = 0;
		gbc_lblTimeText.gridy = 4;
		optionsPanel.add(lblTimeText, gbc_lblTimeText);

		lblTime = new JLabel("");
		GridBagConstraints gbc_lblTime = new GridBagConstraints();
		gbc_lblTime.insets = new Insets(0, 0, 5, 0);
		gbc_lblTime.gridx = 0;
		gbc_lblTime.gridy = 5;
		optionsPanel.add(lblTime, gbc_lblTime);
		
		JLabel lblEventText = new JLabel("Event");
		GridBagConstraints gbc_lblEventText = new GridBagConstraints();
		gbc_lblEventText.insets = new Insets(0, 0, 5, 0);
		gbc_lblEventText.gridx = 0;
		gbc_lblEventText.gridy = 6;
		optionsPanel.add(lblEventText, gbc_lblEventText);
		
		lblEvent = new JLabel("");
		GridBagConstraints gbc_lblEvent = new GridBagConstraints();
		gbc_lblEvent.insets = new Insets(0, 0, 5, 0);
		gbc_lblEvent.gridx = 0;
		gbc_lblEvent.gridy = 7;
		optionsPanel.add(lblEvent, gbc_lblEvent);
		
		JSeparator separator_3 = new JSeparator();
		GridBagConstraints gbc_separator_3 = new GridBagConstraints();
		gbc_separator_3.fill = GridBagConstraints.HORIZONTAL;
		gbc_separator_3.insets = new Insets(0, 0, 5, 0);
		gbc_separator_3.gridx = 0;
		gbc_separator_3.gridy = 8;
		optionsPanel.add(separator_3, gbc_separator_3);

		JLabel lblProductsText = new JLabel("Products");
		GridBagConstraints gbc_lblProductsText = new GridBagConstraints();
		gbc_lblProductsText.insets = new Insets(0, 0, 5, 0);
		gbc_lblProductsText.gridx = 0;
		gbc_lblProductsText.gridy = 9;
		optionsPanel.add(lblProductsText, gbc_lblProductsText);

		JLabel lblProductsSubText = new JLabel("in Grid (created) (steps)");
		GridBagConstraints gbc_lblProductsSubText = new GridBagConstraints();
		gbc_lblProductsSubText.insets = new Insets(0, 0, 5, 0);
		gbc_lblProductsSubText.gridx = 0;
		gbc_lblProductsSubText.gridy = 10;
		optionsPanel.add(lblProductsSubText, gbc_lblProductsSubText);

		lblProducts = new JLabel("");
		GridBagConstraints gbc_lblProducts = new GridBagConstraints();
		gbc_lblProducts.insets = new Insets(0, 0, 5, 0);
		gbc_lblProducts.gridx = 0;
		gbc_lblProducts.gridy = 11;
		optionsPanel.add(lblProducts, gbc_lblProducts);

		JLabel lblTravelingText = new JLabel("Traveling");
		GridBagConstraints gbc_lblTravelingText = new GridBagConstraints();
		gbc_lblTravelingText.insets = new Insets(0, 0, 5, 0);
		gbc_lblTravelingText.gridx = 0;
		gbc_lblTravelingText.gridy = 12;
		optionsPanel.add(lblTravelingText, gbc_lblTravelingText);

		lblTraveling = new JLabel("");
		GridBagConstraints gbc_lblTraveling = new GridBagConstraints();
		gbc_lblTraveling.insets = new Insets(0, 0, 5, 0);
		gbc_lblTraveling.gridx = 0;
		gbc_lblTraveling.gridy = 13;
		optionsPanel.add(lblTraveling, gbc_lblTraveling);

		JLabel lblWaitingTimeText = new JLabel("Avg waiting");
		GridBagConstraints gbc_lblWaitingTimeText = new GridBagConstraints();
		gbc_lblWaitingTimeText.insets = new Insets(0, 0, 5, 0);
		gbc_lblWaitingTimeText.gridx = 0;
		gbc_lblWaitingTimeText.gridy = 14;
		optionsPanel.add(lblWaitingTimeText, gbc_lblWaitingTimeText);

		lblWaitingTime = new JLabel("");
		GridBagConstraints gbc_lblWaitingTime = new GridBagConstraints();
		gbc_lblWaitingTime.insets = new Insets(0, 0, 5, 0);
		gbc_lblWaitingTime.gridx = 0;
		gbc_lblWaitingTime.gridy = 15;
		optionsPanel.add(lblWaitingTime, gbc_lblWaitingTime);

		JLabel lblBusyText = new JLabel("Busy");
		GridBagConstraints gbc_lblBusyText = new GridBagConstraints();
		gbc_lblBusyText.insets = new Insets(0, 0, 5, 0);
		gbc_lblBusyText.gridx = 0;
		gbc_lblBusyText.gridy = 16;
		optionsPanel.add(lblBusyText, gbc_lblBusyText);

		lblBusy = new JLabel("");
		GridBagConstraints gbc_lblBusy = new GridBagConstraints();
		gbc_lblBusy.insets = new Insets(0, 0, 5, 0);
		gbc_lblBusy.gridx = 0;
		gbc_lblBusy.gridy = 17;
		optionsPanel.add(lblBusy, gbc_lblBusy);

		JLabel lblThroughputText = new JLabel("Avg throughput");
		GridBagConstraints gbc_lblThroughputText = new GridBagConstraints();
		gbc_lblThroughputText.insets = new Insets(0, 0, 5, 0);
		gbc_lblThroughputText.gridx = 0;
		gbc_lblThroughputText.gridy = 18;
		optionsPanel.add(lblThroughputText, gbc_lblThroughputText);

		lblThroughput = new JLabel("");
		GridBagConstraints gbc_lblThroughput = new GridBagConstraints();
		gbc_lblThroughput.insets = new Insets(0, 0, 5, 0);
		gbc_lblThroughput.gridx = 0;
		gbc_lblThroughput.gridy = 19;
		optionsPanel.add(lblThroughput, gbc_lblThroughput);

		Component verticalStrut = Box.createVerticalStrut(20);
		GridBagConstraints gbc_verticalStrut = new GridBagConstraints();
		gbc_verticalStrut.fill = GridBagConstraints.VERTICAL;
		gbc_verticalStrut.insets = new Insets(0, 0, 5, 0);
		gbc_verticalStrut.gridx = 0;
		gbc_verticalStrut.gridy = 20;
		optionsPanel.add(verticalStrut, gbc_verticalStrut);

		JSeparator separator = new JSeparator();
		GridBagConstraints gbc_separator = new GridBagConstraints();
		gbc_separator.fill = GridBagConstraints.HORIZONTAL;
		gbc_separator.insets = new Insets(0, 0, 5, 0);
		gbc_separator.gridx = 0;
		gbc_separator.gridy = 21;
		optionsPanel.add(separator, gbc_separator);

		JPanel panel = new JPanel();
		GridBagConstraints gbc_panel = new GridBagConstraints();
		gbc_panel.insets = new Insets(0, 0, 5, 0);
		gbc_panel.fill = GridBagConstraints.HORIZONTAL;
		gbc_panel.gridx = 0;
		gbc_panel.gridy = 22;
		optionsPanel.add(panel, gbc_panel);

		JLabel lblDelayText = new JLabel("Delay");
		panel.add(lblDelayText);

		final JLabel lblDelay = new JLabel(String.valueOf(simulation.getDelay()));
		panel.add(lblDelay);

		final JSlider slider = new JSlider(0, 2000, simulation.getDelay());
		slider.setSnapToTicks(true);
		slider.setMajorTickSpacing(100);
		slider.setMinorTickSpacing(10);
		slider.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent e) {
				lblDelay.setText(String.valueOf(slider.getValue()));
				simulation.setDelay(slider.getValue());
			}
		});
		GridBagConstraints gbc_slider = new GridBagConstraints();
		gbc_slider.insets = new Insets(0, 0, 5, 0);
		gbc_slider.fill = GridBagConstraints.HORIZONTAL;
		gbc_slider.gridx = 0;
		gbc_slider.gridy = 23;
		optionsPanel.add(slider, gbc_slider);

		JSeparator separator_2 = new JSeparator();
		GridBagConstraints gbc_separator_2 = new GridBagConstraints();
		gbc_separator_2.fill = GridBagConstraints.HORIZONTAL;
		gbc_separator_2.insets = new Insets(0, 0, 10, 0);
		gbc_separator_2.gridx = 0;
		gbc_separator_2.gridy = 24;
		optionsPanel.add(separator_2, gbc_separator_2);

		JButton btnSave = new JButton("Save");
		btnSave.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				simulation.saveStatistics();
			}
		});
		GridBagConstraints gbc_btnSave = new GridBagConstraints();
		gbc_btnSave.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnSave.insets = new Insets(0, 15, 0, 15);
		gbc_btnSave.gridx = 0;
		gbc_btnSave.gridy = 25;
		optionsPanel.add(btnSave, gbc_btnSave);

		frmRexosSimulation.getRootPane().setDefaultButton(btnStart);
		btnStart.requestFocus();
		btnStart.requestFocusInWindow();
		frmRexosSimulation.setVisible(true);
	}

	public void reset() {
		btnStart.setText("Start");
	}

	public void update(double time, String event, int products, int productCount, int totalSteps, int traveling, List<Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>> equipletStates, double waitingTime, List<Double> busy, double throughput) {
		String[] busyValues = new String[busy.size()];
		for (int i = 0; i < busy.size(); i++) {
			busyValues[i] = String.format("%.0f%%", busy.get(i));
		}

		NumberFormat nf = NumberFormat.getInstance(Locale.GERMAN);
		nf.setMaximumFractionDigits(1);

		lblTime.setText(nf.format(time));
		lblEvent.setText(event);
		lblProducts.setText(String.format("%d (%d) (%d)", products, productCount, totalSteps));
		lblTraveling.setText(String.format("%d", traveling));
		lblWaitingTime.setText(String.format("%.2f", waitingTime));
		lblBusy.setText(Arrays.toString(busyValues));
		lblThroughput.setText(String.format("%.2f", throughput));
		gridView.update(equipletStates);
	}

	@Deprecated
	public void update(double time, int products, int productCount, int totalSteps, Map<String, Pair<Position, Tuple<String, Integer, Integer, Integer>>> equiplets, double waitingTime, List<Double> busy, double throughput) {
		String[] busyValues = new String[busy.size()];
		for (int i = 0; i < busy.size(); i++) {
			busyValues[i] = String.format("%.0f%%", busy.get(i));
		}

		NumberFormat nf = NumberFormat.getInstance(Locale.GERMAN);
		nf.setMaximumFractionDigits(1);

		lblTime.setText(nf.format(time));
		lblProducts.setText(String.format("%d (%d) (%d)", products, productCount, totalSteps));
		lblTraveling.setText(String.format("%d", 0));
		lblWaitingTime.setText(String.format("%.2f", waitingTime));
		lblBusy.setText(Arrays.toString(busyValues));
		lblThroughput.setText(String.format("%.2f", throughput));
		gridView.update(equiplets);
	}

	@Deprecated
	public void update(double time, int products, int productCount, int totalSteps, Collection<Equiplet> equiplets, double waitingTime, List<Double> busy, double throughput) {
		String[] busyValues = new String[busy.size()];
		for (int i = 0; i < busy.size(); i++) {
			busyValues[i] = String.format("%.0f%%", busy.get(i));
		}

		NumberFormat nf = NumberFormat.getInstance(Locale.GERMAN);
		nf.setMaximumFractionDigits(1);

		lblTime.setText(nf.format(time));
		lblProducts.setText(String.format("%d (%d) (%d)", products, productCount, totalSteps));
		lblTraveling.setText(String.format("%d", 0));
		lblWaitingTime.setText(String.format("%.2f", waitingTime));
		lblBusy.setText(Arrays.toString(busyValues));
		lblThroughput.setText(String.format("%.2f", throughput));
		// gridView.update(equiplets);

	}
}
