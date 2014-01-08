package simulation.gui;

import java.awt.Dialog.ModalityType;
import java.awt.Dimension;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.border.TitledBorder;

import simulation.data.BatchDescription;
import simulation.data.Capability;
import simulation.data.EquipletDescription;
import simulation.data.ProductDescription;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;

public class ConfigurationEditorGUI extends JFrame {
	/**
	 * 
	 */
	private static final long serialVersionUID = 5558864594487651654L;

	Window win;
	
	public DefaultListModel<Capability> capabilities = new DefaultListModel<Capability>();
	public DefaultListModel<BatchDescription> batches = new DefaultListModel<BatchDescription>();
	public DefaultListModel<ProductDescription> products = new DefaultListModel<ProductDescription>();
	public DefaultListModel<EquipletDescription> equiplets = new DefaultListModel<EquipletDescription>();
	public DefaultListModel<EquipletDescription> gridContent = new DefaultListModel<EquipletDescription>();
	public EquipletDescription[][] grid;

	JPanel capPanel = new JPanel();

	public ConfigurationEditorGUI() {
		setTitle("Configuration Editor");
		setSize(new Dimension(800, 415));
		setLocationRelativeTo(null);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
		
		JPanel panel = new JPanel();
		getContentPane().add(panel);
				GridBagLayout gbl_panel = new GridBagLayout();
				gbl_panel.columnWidths = new int[]{255, 255, 255, 0};
				gbl_panel.rowHeights = new int[]{180, 180, 0};
				gbl_panel.columnWeights = new double[]{0.0, 0.0, 0.0, Double.MIN_VALUE};
				gbl_panel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
				panel.setLayout(gbl_panel);
										GridBagConstraints gbc_capPanel = new GridBagConstraints();
										gbc_capPanel.fill = GridBagConstraints.BOTH;
										gbc_capPanel.insets = new Insets(0, 0, 5, 5);
										gbc_capPanel.gridx = 0;
										gbc_capPanel.gridy = 0;
										panel.add(capPanel, gbc_capPanel);
										
												capPanel.setBorder(new TitledBorder(null, "Capability",
														TitledBorder.LEADING, TitledBorder.TOP, null, null));
												win = SwingUtilities.getWindowAncestor(capPanel);
												GridBagLayout gbl_capPanel = new GridBagLayout();
												gbl_capPanel.columnWidths = new int[]{250, 0};
												gbl_capPanel.rowHeights = new int[]{25, 133, 0};
												gbl_capPanel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
												gbl_capPanel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
												capPanel.setLayout(gbl_capPanel);
														

														JButton addCap = new JButton("Create Capability");
														addCap.addActionListener(new ActionListener() {
															public void actionPerformed(ActionEvent arg0) {
																showCapabilityDialog();
															}
														});
														GridBagConstraints gbc_addCap = new GridBagConstraints();
														gbc_addCap.fill = GridBagConstraints.HORIZONTAL;
														gbc_addCap.insets = new Insets(0, 0, 5, 0);
														gbc_addCap.gridx = 0;
														gbc_addCap.gridy = 0;
														capPanel.add(addCap, gbc_addCap);
												
														JList<Capability> capList = new JList<Capability>();
														capList.setModel(capabilities);
														capList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
														
																JScrollPane capListPane = new JScrollPane(capList);
																GridBagConstraints gbc_capListPane = new GridBagConstraints();
																gbc_capListPane.fill = GridBagConstraints.BOTH;
																gbc_capListPane.gridx = 0;
																gbc_capListPane.gridy = 1;
																capPanel.add(capListPane, gbc_capListPane);
												
														JPanel prodPanel = new JPanel();
														GridBagConstraints gbc_prodPanel = new GridBagConstraints();
														gbc_prodPanel.fill = GridBagConstraints.BOTH;
														gbc_prodPanel.insets = new Insets(0, 0, 5, 5);
														gbc_prodPanel.gridx = 1;
														gbc_prodPanel.gridy = 0;
														panel.add(prodPanel, gbc_prodPanel);
														prodPanel.setBorder(new TitledBorder(null, "Product",
																TitledBorder.LEADING, TitledBorder.TOP, null, null));
																GridBagLayout gbl_prodPanel = new GridBagLayout();
																gbl_prodPanel.columnWidths = new int[]{250, 0};
																gbl_prodPanel.rowHeights = new int[]{25, 133, 0};
																gbl_prodPanel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
																gbl_prodPanel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
																prodPanel.setLayout(gbl_prodPanel);
																		
																				JButton addProd = new JButton("Create Product");
																				addProd.addActionListener(new ActionListener() {
																					public void actionPerformed(ActionEvent arg0) {
																						showProductDialog();
																					}
																				});
																				GridBagConstraints gbc_addProd = new GridBagConstraints();
																				gbc_addProd.fill = GridBagConstraints.HORIZONTAL;
																				gbc_addProd.insets = new Insets(0, 0, 5, 0);
																				gbc_addProd.gridx = 0;
																				gbc_addProd.gridy = 0;
																				prodPanel.add(addProd, gbc_addProd);
																				
																						JList<ProductDescription> productList = new JList<ProductDescription>();
																						productList.setModel(products);
																						productList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
																						
																								JScrollPane productListPane = new JScrollPane(productList);
																								GridBagConstraints gbc_productListPane = new GridBagConstraints();
																								gbc_productListPane.fill = GridBagConstraints.BOTH;
																								gbc_productListPane.gridx = 0;
																								gbc_productListPane.gridy = 1;
																								prodPanel.add(productListPane, gbc_productListPane);
										
												JPanel batchPanel = new JPanel();
												GridBagConstraints gbc_batchPanel = new GridBagConstraints();
												gbc_batchPanel.fill = GridBagConstraints.BOTH;
												gbc_batchPanel.insets = new Insets(0, 0, 5, 0);
												gbc_batchPanel.gridx = 2;
												gbc_batchPanel.gridy = 0;
												panel.add(batchPanel, gbc_batchPanel);
												batchPanel.setBorder(new TitledBorder(null, "Batch",
														TitledBorder.LEADING, TitledBorder.TOP, null, null));
														GridBagLayout gbl_batchPanel = new GridBagLayout();
														gbl_batchPanel.columnWidths = new int[]{250, 0};
														gbl_batchPanel.rowHeights = new int[]{25, 133, 0};
														gbl_batchPanel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
														gbl_batchPanel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
														batchPanel.setLayout(gbl_batchPanel);
																
																		JButton addBatch = new JButton("Create Batch");
																		addBatch.addActionListener(new ActionListener() {
																			public void actionPerformed(ActionEvent arg0) {
																				showBatchDialog();
																			}
																		});
																		GridBagConstraints gbc_addBatch = new GridBagConstraints();
																		gbc_addBatch.fill = GridBagConstraints.HORIZONTAL;
																		gbc_addBatch.insets = new Insets(0, 0, 5, 0);
																		gbc_addBatch.gridx = 0;
																		gbc_addBatch.gridy = 0;
																		batchPanel.add(addBatch, gbc_addBatch);
														
																JList<BatchDescription> batchList = new JList<BatchDescription>();
																batchList.setModel(batches);
																batchList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
																
																		JScrollPane batchListPane = new JScrollPane(batchList);
																		GridBagConstraints gbc_batchListPane = new GridBagConstraints();
																		gbc_batchListPane.fill = GridBagConstraints.BOTH;
																		gbc_batchListPane.gridx = 0;
																		gbc_batchListPane.gridy = 1;
																		batchPanel.add(batchListPane, gbc_batchListPane);
												
														JPanel eqPanel = new JPanel();
														GridBagConstraints gbc_eqPanel = new GridBagConstraints();
														gbc_eqPanel.fill = GridBagConstraints.BOTH;
														gbc_eqPanel.insets = new Insets(0, 0, 0, 5);
														gbc_eqPanel.gridx = 0;
														gbc_eqPanel.gridy = 1;
														panel.add(eqPanel, gbc_eqPanel);
														eqPanel.setBorder(new TitledBorder(null, "Equiplet",
																TitledBorder.LEADING, TitledBorder.TOP, null, null));
																GridBagLayout gbl_eqPanel = new GridBagLayout();
																gbl_eqPanel.columnWidths = new int[]{250, 0};
																gbl_eqPanel.rowHeights = new int[]{25, 133, 0};
																gbl_eqPanel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
																gbl_eqPanel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
																eqPanel.setLayout(gbl_eqPanel);
																		
																				JButton addEquiplet = new JButton("Create Equiplet");
																				addEquiplet.addActionListener(new ActionListener() {
																					public void actionPerformed(ActionEvent arg0) {
																						showEquipletDialog();
																					}
																				});
																				GridBagConstraints gbc_addEquiplet = new GridBagConstraints();
																				gbc_addEquiplet.fill = GridBagConstraints.HORIZONTAL;
																				gbc_addEquiplet.insets = new Insets(0, 0, 5, 0);
																				gbc_addEquiplet.gridx = 0;
																				gbc_addEquiplet.gridy = 0;
																				eqPanel.add(addEquiplet, gbc_addEquiplet);
																
																		JList<EquipletDescription> eqList = new JList<EquipletDescription>();
																		eqList.setModel(equiplets);
																		eqList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
																		
																				JScrollPane eqListPane = new JScrollPane(eqList);
																				GridBagConstraints gbc_eqListPane = new GridBagConstraints();
																				gbc_eqListPane.fill = GridBagConstraints.BOTH;
																				gbc_eqListPane.gridx = 0;
																				gbc_eqListPane.gridy = 1;
																				eqPanel.add(eqListPane, gbc_eqListPane);
										
												JPanel gridPanel = new JPanel();
												GridBagConstraints gbc_gridPanel = new GridBagConstraints();
												gbc_gridPanel.fill = GridBagConstraints.BOTH;
												gbc_gridPanel.insets = new Insets(0, 0, 0, 5);
												gbc_gridPanel.gridx = 1;
												gbc_gridPanel.gridy = 1;
												panel.add(gridPanel, gbc_gridPanel);
												gridPanel.setBorder(new TitledBorder(null, "Grid",
														TitledBorder.LEADING, TitledBorder.TOP, null, null));
														GridBagLayout gbl_gridPanel = new GridBagLayout();
														gbl_gridPanel.columnWidths = new int[]{250, 0};
														gbl_gridPanel.rowHeights = new int[]{25, 133, 0};
														gbl_gridPanel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
														gbl_gridPanel.rowWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
														gridPanel.setLayout(gbl_gridPanel);
														
																JButton addGrid = new JButton("Create Grid");
																addGrid.addActionListener(new ActionListener() {
																	public void actionPerformed(ActionEvent arg0) {
																		showGridDialog();
																	}
																});
																GridBagConstraints gbc_addGrid = new GridBagConstraints();
																gbc_addGrid.fill = GridBagConstraints.HORIZONTAL;
																gbc_addGrid.insets = new Insets(0, 0, 5, 0);
																gbc_addGrid.gridx = 0;
																gbc_addGrid.gridy = 0;
																gridPanel.add(addGrid, gbc_addGrid);
														
														JList<EquipletDescription> gridList = new JList<EquipletDescription>();
														gridList.setModel(gridContent);
														gridList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
														
																JScrollPane gridListPane = new JScrollPane(gridList);
																GridBagConstraints gbc_gridListPane = new GridBagConstraints();
																gbc_gridListPane.fill = GridBagConstraints.BOTH;
																gbc_gridListPane.gridx = 0;
																gbc_gridListPane.gridy = 1;
																gridPanel.add(gridListPane, gbc_gridListPane);
																										
																										JPanel buttonPanel = new JPanel();
																										buttonPanel.setBorder(new TitledBorder(null, "Export", TitledBorder.LEADING, TitledBorder.TOP, null, null));
																										GridBagConstraints gbc_buttonPanel = new GridBagConstraints();
																										gbc_buttonPanel.fill = GridBagConstraints.BOTH;
																										gbc_buttonPanel.gridx = 2;
																										gbc_buttonPanel.gridy = 1;
																										panel.add(buttonPanel, gbc_buttonPanel);
																										GridBagLayout gbl_buttonPanel = new GridBagLayout();
																										gbl_buttonPanel.columnWidths = new int[] {50, 116, 0};
																										gbl_buttonPanel.rowHeights = new int[] {25, 25, 25, 0};
																										gbl_buttonPanel.columnWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
																										gbl_buttonPanel.rowWeights = new double[]{0.0, 0.0, 0.0, Double.MIN_VALUE};
																										buttonPanel.setLayout(gbl_buttonPanel);
																										
																										JButton exportButton = new JButton("Export Files");
																										exportButton.addActionListener(new ActionListener() {
																											public void actionPerformed(ActionEvent e) {
																												writeBatchesCSV();
																												writeCapabilitiesCSV();
																												writeGridJSON();
																												writeProductsCSV();
																											}
																										});
																										GridBagConstraints gbc_exportButton = new GridBagConstraints();
																										gbc_exportButton.gridx = 1;
																										gbc_exportButton.gridy = 2;
																										buttonPanel.add(exportButton, gbc_exportButton);
	}

	protected void showCapabilityDialog() {
		if (win != null) {
			CreateCapabilityDialog dialog = new CreateCapabilityDialog(win, "Create new capability", ModalityType.APPLICATION_MODAL);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				capabilities.addElement(dialog.getCapability());
			}
		}
	}

	protected void showBatchDialog() {
		if (win != null) {
			CreateBatchDialog dialog = new CreateBatchDialog(win, "Create new batch", ModalityType.APPLICATION_MODAL, products);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				batches.addElement(dialog.getBatch());
			}
		}
	}

	protected void showEquipletDialog() {
		if (win != null) {
			CreateEquipletDialog dialog = new CreateEquipletDialog(win, "Create new equiplet", ModalityType.APPLICATION_MODAL, capabilities, batches);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				equiplets.addElement(dialog.getEquiplet());
			}
		}
	}

	protected void showProductDialog() {
		if (win != null) {
			CreateProductDialog dialog = new CreateProductDialog(win, "Create new product", ModalityType.APPLICATION_MODAL, capabilities);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				products.addElement(dialog.getProduct());
			}
		}
	}
	
	protected void showGridDialog() {
		if (win != null) {
			CreateGridDialog dialog = new CreateGridDialog(win, "Assemble Grid", ModalityType.APPLICATION_MODAL, equiplets);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				gridContent.clear();
				grid = dialog.getGrid();
				
				for(int i = 0; i < grid.length; i++) {
					for(int j = 0; j < grid[i].length; j++) {
						gridContent.addElement(grid[i][j]);
					}
				}
			}
		}
	}
	
	private void writeProductsCSV() {
		try {
			PrintWriter writer = new PrintWriter("products.csv", "UTF-8");
			
			for (int i = 0; i < products.getSize(); i++) {
				writer.write(products.getElementAt(i).toCsvString());
			}
			
			writer.flush();
			writer.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
	}
	
	private void writeGridJSON() {
		try {
			PrintWriter writer = new PrintWriter("grid.json", "UTF-8");
			
			writer.write("{\n"
					+ "\t\"grid\" : [\n");
			
			for (int i = 0; i < grid.length; i++) {
				writer.write("\t\t[\n");
				
				for(int j = 0; j < grid[i].length; j++) {
					writer.write(grid[i][j].toJsonString());
					
					if(j < grid[i].length - 1) {
						writer.write("\n\t\t\t,\n");
					}
				}
				
				writer.write("\n\t\t]\n");
				
				if(i < grid.length - 1) {
					writer.write("\t\t,\n");
				}
			}
			
			writer.write("\n}");
			
			writer.flush();
			writer.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
	}
	
	private void writeCapabilitiesCSV() {
		try {
			PrintWriter writer = new PrintWriter("capabilities.csv", "UTF-8");
			
			for (int i = 0; i < capabilities.getSize(); i++) {
				writer.write(capabilities.getElementAt(i).toCsvString());
			}
			
			writer.flush();
			writer.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
	}
	
	private void writeBatchesCSV() {
		try {
			PrintWriter writer = new PrintWriter("batches.csv", "UTF-8");
			
			for (int i = 0; i < batches.getSize(); i++) {
				writer.write(batches.getElementAt(i).toCsvString());
			}
			
			writer.flush();
			writer.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args) {
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				ConfigurationEditorGUI gui = new ConfigurationEditorGUI();
				gui.setVisible(true);
			}
		});
	}
}
