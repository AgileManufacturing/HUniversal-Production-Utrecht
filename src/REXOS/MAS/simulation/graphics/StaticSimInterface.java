package MAS.simulation.graphics;

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
import javax.swing.ScrollPaneConstants;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import MAS.simulation.offline.StaticSimulation;
import MAS.util.Triple;

public class StaticSimInterface {
// dan maar niet
//	private JFrame frmRexosSimulation;
//	private JLabel lblTime;
//	private GridView gridView;
//	private JPanel chartPanel;
//
//	private StaticSimulation offlineSim;
//	private JLabel lblWaitingTime;
//	private JLabel lblBusy;
//	private JLabel lblTraveling;
//	private JLabel lblProducts;
//	private JLabel lblThroughput;
//	private JButton btnStart;
//
//	/**
//	 * Launch the application.
//	 */
//	public static void main(String[] args) {
//		EventQueue.invokeLater(new Runnable() {
//			public void run() {
//				try {
//					StaticSimInterface window = new StaticSimInterface();
//					window.frmRexosSimulation.setVisible(true);
//				} catch (Exception e) {
//					e.printStackTrace();
//				}
//			}
//		});
//	}
//
//	/**
//	 * Create the application.
//	 */
//	public StaticSimInterface() {
//		init();
//		initialize();
//	}
//
//	/**
//	 * Initialize the contents of the frame.
//	 */
//	private void initialize() {
//		frmRexosSimulation = new JFrame();
//		frmRexosSimulation.setTitle("REXOS Simulation");
//		frmRexosSimulation.setBounds(100, 100, 800, 500);
//		frmRexosSimulation.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//
//		JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.TOP);
//		frmRexosSimulation.getContentPane().add(tabbedPane, BorderLayout.CENTER);
//
//		gridView = new GridView();
//		gridView.setBackground(Color.WHITE);
//		tabbedPane.addTab("Grid", null, gridView, null);
//		tabbedPane.setFocusable(false);
//
//		JScrollPane scrollPane = new JScrollPane();
//		scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);
//		tabbedPane.addTab("Schedule", null, scrollPane, null);
//
//		chartPanel = new JPanel();
//		scrollPane.setViewportView(chartPanel);
//		chartPanel.setLayout(new BoxLayout(chartPanel, BoxLayout.X_AXIS));
//
//		final JPanel timeDivisionView = new JPanel();
//		tabbedPane.addTab("Equiplet time division", null, timeDivisionView, null);
//		timeDivisionView.setLayout(new BoxLayout(timeDivisionView, BoxLayout.X_AXIS));
//
//		ChangeListener changeListener = new ChangeListener() {
//			public void stateChanged(ChangeEvent changeEvent) {
//				JTabbedPane sourceTabbedPane = (JTabbedPane) changeEvent.getSource();
//				int index = sourceTabbedPane.getSelectedIndex();
//				if (index == 2) {
//					Map<String, Triple<Double, Double, Double>> history = offlineSim.getEquipletHistory();
//					System.out.println("HISTORY: " + history);
//					timeDivisionView.removeAll();
//					// timeDivisionView.add(StackedBarChart.createChartPanel(history));
//				}
//				System.out.println("Tab changed to: " + sourceTabbedPane.getTitleAt(index));
//			}
//		};
//		tabbedPane.addChangeListener(changeListener);
//
//		JPanel optionsPanel = new JPanel();
//		frmRexosSimulation.getContentPane().add(optionsPanel, BorderLayout.EAST);
//		GridBagLayout gbl_optionsPanel = new GridBagLayout();
//		gbl_optionsPanel.columnWidths = new int[] { 70, 0 };
//		gbl_optionsPanel.rowHeights = new int[] { 15, 25, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 462, 0, 0, 0, 0, 0, 0 };
//		gbl_optionsPanel.columnWeights = new double[] { 1.0, Double.MIN_VALUE };
//		gbl_optionsPanel.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
//		optionsPanel.setLayout(gbl_optionsPanel);
//
//		btnStart = new JButton("Start");
//		btnStart.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent e) {
//				if (btnStart.getText().matches("Start")) {
//					btnStart.setText("Pause");
//				} else {
//					btnStart.setText("Start");
//				}
//				offlineSim.pause();
//			}
//		});
//
//		JLabel lblOptions = new JLabel("Options");
//		lblOptions.setHorizontalAlignment(SwingConstants.CENTER);
//		GridBagConstraints gbc_lblOptions = new GridBagConstraints();
//		gbc_lblOptions.anchor = GridBagConstraints.WEST;
//		gbc_lblOptions.insets = new Insets(0, 0, 5, 0);
//		gbc_lblOptions.gridx = 0;
//		gbc_lblOptions.gridy = 0;
//		optionsPanel.add(lblOptions, gbc_lblOptions);
//		GridBagConstraints gbc_btnStart = new GridBagConstraints();
//		gbc_btnStart.fill = GridBagConstraints.HORIZONTAL;
//		gbc_btnStart.insets = new Insets(0, 15, 5, 15);
//		gbc_btnStart.gridx = 0;
//		gbc_btnStart.gridy = 1;
//		optionsPanel.add(btnStart, gbc_btnStart);
//
//		JSeparator separator_1 = new JSeparator();
//		GridBagConstraints gbc_separator_1 = new GridBagConstraints();
//		gbc_separator_1.fill = GridBagConstraints.HORIZONTAL;
//		gbc_separator_1.insets = new Insets(0, 0, 5, 0);
//		gbc_separator_1.gridx = 0;
//		gbc_separator_1.gridy = 2;
//		optionsPanel.add(separator_1, gbc_separator_1);
//
//		JLabel lblTimeText = new JLabel("Time");
//		lblTimeText.setHorizontalAlignment(SwingConstants.CENTER);
//		GridBagConstraints gbc_lblTimeText = new GridBagConstraints();
//		gbc_lblTimeText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblTimeText.anchor = GridBagConstraints.WEST;
//		gbc_lblTimeText.gridx = 0;
//		gbc_lblTimeText.gridy = 3;
//		optionsPanel.add(lblTimeText, gbc_lblTimeText);
//
//		lblTime = new JLabel("");
//		GridBagConstraints gbc_lblTime = new GridBagConstraints();
//		gbc_lblTime.insets = new Insets(0, 0, 5, 0);
//		gbc_lblTime.gridx = 0;
//		gbc_lblTime.gridy = 4;
//		optionsPanel.add(lblTime, gbc_lblTime);
//
//		JLabel lblProductsText = new JLabel("Products");
//		GridBagConstraints gbc_lblProductsText = new GridBagConstraints();
//		gbc_lblProductsText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblProductsText.gridx = 0;
//		gbc_lblProductsText.gridy = 5;
//		optionsPanel.add(lblProductsText, gbc_lblProductsText);
//
//		lblProducts = new JLabel("");
//		GridBagConstraints gbc_lblProducts = new GridBagConstraints();
//		gbc_lblProducts.insets = new Insets(0, 0, 5, 0);
//		gbc_lblProducts.gridx = 0;
//		gbc_lblProducts.gridy = 6;
//		optionsPanel.add(lblProducts, gbc_lblProducts);
//
//		JLabel lblTravelingText = new JLabel("Traveling");
//		GridBagConstraints gbc_lblTravelingText = new GridBagConstraints();
//		gbc_lblTravelingText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblTravelingText.gridx = 0;
//		gbc_lblTravelingText.gridy = 7;
//		optionsPanel.add(lblTravelingText, gbc_lblTravelingText);
//
//		lblTraveling = new JLabel("");
//		GridBagConstraints gbc_lblTraveling = new GridBagConstraints();
//		gbc_lblTraveling.insets = new Insets(0, 0, 5, 0);
//		gbc_lblTraveling.gridx = 0;
//		gbc_lblTraveling.gridy = 8;
//		optionsPanel.add(lblTraveling, gbc_lblTraveling);
//
//		JLabel lblWaitingTimeText = new JLabel("Avg waiting");
//		GridBagConstraints gbc_lblWaitingTimeText = new GridBagConstraints();
//		gbc_lblWaitingTimeText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblWaitingTimeText.gridx = 0;
//		gbc_lblWaitingTimeText.gridy = 9;
//		optionsPanel.add(lblWaitingTimeText, gbc_lblWaitingTimeText);
//
//		lblWaitingTime = new JLabel("");
//		GridBagConstraints gbc_lblWaitingTime = new GridBagConstraints();
//		gbc_lblWaitingTime.insets = new Insets(0, 0, 5, 0);
//		gbc_lblWaitingTime.gridx = 0;
//		gbc_lblWaitingTime.gridy = 10;
//		optionsPanel.add(lblWaitingTime, gbc_lblWaitingTime);
//
//		JLabel lblBusyText = new JLabel("Busy");
//		GridBagConstraints gbc_lblBusyText = new GridBagConstraints();
//		gbc_lblBusyText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblBusyText.gridx = 0;
//		gbc_lblBusyText.gridy = 11;
//		optionsPanel.add(lblBusyText, gbc_lblBusyText);
//
//		lblBusy = new JLabel("");
//		GridBagConstraints gbc_lblBusy = new GridBagConstraints();
//		gbc_lblBusy.insets = new Insets(0, 0, 5, 0);
//		gbc_lblBusy.gridx = 0;
//		gbc_lblBusy.gridy = 12;
//		optionsPanel.add(lblBusy, gbc_lblBusy);
//
//		JLabel lblThroughputText = new JLabel("Avg throughput");
//		GridBagConstraints gbc_lblThroughputText = new GridBagConstraints();
//		gbc_lblThroughputText.insets = new Insets(0, 0, 5, 0);
//		gbc_lblThroughputText.gridx = 0;
//		gbc_lblThroughputText.gridy = 13;
//		optionsPanel.add(lblThroughputText, gbc_lblThroughputText);
//
//		lblThroughput = new JLabel("");
//		GridBagConstraints gbc_lblThroughput = new GridBagConstraints();
//		gbc_lblThroughput.insets = new Insets(0, 0, 5, 0);
//		gbc_lblThroughput.gridx = 0;
//		gbc_lblThroughput.gridy = 14;
//		optionsPanel.add(lblThroughput, gbc_lblThroughput);
//
//		Component verticalStrut = Box.createVerticalStrut(20);
//		GridBagConstraints gbc_verticalStrut = new GridBagConstraints();
//		gbc_verticalStrut.fill = GridBagConstraints.VERTICAL;
//		gbc_verticalStrut.insets = new Insets(0, 0, 5, 0);
//		gbc_verticalStrut.gridx = 0;
//		gbc_verticalStrut.gridy = 15;
//		optionsPanel.add(verticalStrut, gbc_verticalStrut);
//
//		JSeparator separator = new JSeparator();
//		GridBagConstraints gbc_separator = new GridBagConstraints();
//		gbc_separator.fill = GridBagConstraints.HORIZONTAL;
//		gbc_separator.insets = new Insets(0, 0, 5, 0);
//		gbc_separator.gridx = 0;
//		gbc_separator.gridy = 16;
//		optionsPanel.add(separator, gbc_separator);
//
//		JPanel panel = new JPanel();
//		GridBagConstraints gbc_panel = new GridBagConstraints();
//		gbc_panel.insets = new Insets(0, 0, 5, 0);
//		gbc_panel.fill = GridBagConstraints.HORIZONTAL;
//		gbc_panel.gridx = 0;
//		gbc_panel.gridy = 17;
//		optionsPanel.add(panel, gbc_panel);
//
//		JLabel lblDelayText = new JLabel("Delay");
//		panel.add(lblDelayText);
//
//		final JLabel lblDelay = new JLabel(String.valueOf(offlineSim.getDelay()));
//		panel.add(lblDelay);
//
//		final JSlider slider = new JSlider(0, 2000, offlineSim.getDelay());
//		slider.setSnapToTicks(true);
//		slider.setMajorTickSpacing(100);
//		slider.setMinorTickSpacing(10);
//		slider.addChangeListener(new ChangeListener() {
//			public void stateChanged(ChangeEvent e) {
//				lblDelay.setText(String.valueOf(slider.getValue()));
//				offlineSim.setDelay(slider.getValue());
//			}
//		});
//		GridBagConstraints gbc_slider = new GridBagConstraints();
//		gbc_slider.insets = new Insets(0, 0, 5, 0);
//		gbc_slider.fill = GridBagConstraints.HORIZONTAL;
//		gbc_slider.gridx = 0;
//		gbc_slider.gridy = 18;
//		optionsPanel.add(slider, gbc_slider);
//
//		JSeparator separator_2 = new JSeparator();
//		GridBagConstraints gbc_separator_2 = new GridBagConstraints();
//		gbc_separator_2.fill = GridBagConstraints.HORIZONTAL;
//		gbc_separator_2.insets = new Insets(0, 0, 10, 0);
//		gbc_separator_2.gridx = 0;
//		gbc_separator_2.gridy = 19;
//		optionsPanel.add(separator_2, gbc_separator_2);
//
//		JButton btnSave = new JButton("Save");
//		btnSave.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent arg0) {
//				offlineSim.saveStatistics();
//			}
//		});
//		GridBagConstraints gbc_btnSave = new GridBagConstraints();
//		gbc_btnSave.fill = GridBagConstraints.HORIZONTAL;
//		gbc_btnSave.insets = new Insets(0, 15, 5, 15);
//		gbc_btnSave.gridx = 0;
//		gbc_btnSave.gridy = 20;
//		optionsPanel.add(btnSave, gbc_btnSave);
//
//		frmRexosSimulation.getRootPane().setDefaultButton(btnStart);
//		btnStart.requestFocus();
//		btnStart.requestFocusInWindow();
//	}
//
//	private void init() {
//		offlineSim = new StaticSimulation(this);
//		offlineSim.start();
//	}
//
//	public void reset() {
//		btnStart.setText("Start");
//	}
//
//	public void update(double time, int products, int productCount, int totalSteps, int traveling, List<Triple<String, List<String>, Triple<String, Integer, Integer>>> equipletStates, double waitingTime, List<Double> busy, double throughput) {
//		String[] busyValues = new String[busy.size()];
//		for (int i = 0; i < busy.size(); i++) {
//			busyValues[i] = String.format("%.0f%%", busy.get(i));
//		}
//
//		NumberFormat nf = NumberFormat.getInstance(Locale.GERMAN);
//		nf.setMaximumFractionDigits(1);
//
//		lblTime.setText(nf.format(time));
//		lblProducts.setText(String.format("%d (%d) (%d)", products, productCount, totalSteps));
//		lblTraveling.setText(String.format("%d", traveling));
//		lblWaitingTime.setText(String.format("%.2f", waitingTime));
//		lblBusy.setText(Arrays.toString(busyValues));
//		lblThroughput.setText(String.format("%.2f", throughput));
//		// gridView.update(equipletStates);
//	}
//	*/
}
