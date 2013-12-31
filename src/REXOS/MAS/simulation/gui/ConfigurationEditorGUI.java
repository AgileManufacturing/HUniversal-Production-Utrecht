package simulation.gui;

import jade.util.leap.Iterator;

import java.awt.Dialog.ModalityType;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JDialog;
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
import simulation.mas_entities.Batch;
import simulation.mas_entities.Product;
import javax.swing.SwingConstants;

public class ConfigurationEditorGUI extends JFrame {
	Window win;
	
	DefaultListModel<Capability> capabilities = new DefaultListModel<Capability>();
	DefaultListModel<BatchDescription> batches = new DefaultListModel<BatchDescription>();
	DefaultListModel<ProductDescription> products = new DefaultListModel<ProductDescription>();
	DefaultListModel<EquipletDescription> equiplets = new DefaultListModel<EquipletDescription>();
	DefaultListModel<EquipletDescription> gridContent = new DefaultListModel<EquipletDescription>();

	JPanel capPanel = new JPanel();

	public ConfigurationEditorGUI() {
		setTitle("Configuration Editor");
		setSize(new Dimension(300, 800));
		setLocationRelativeTo(null);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));

		capPanel.setBorder(new TitledBorder(null, "Capability",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(capPanel);
		win = SwingUtilities.getWindowAncestor(capPanel);
		capPanel.setLayout(new BoxLayout(capPanel, BoxLayout.Y_AXIS));
		

		JButton addCap = new JButton("Create Capability");
		addCap.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showCapabilityDialog();
			}
		});
		capPanel.add(addCap);

		JList capList = new JList();
		capList.setModel(capabilities);
		capList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane capListPane = new JScrollPane(capList);
		capPanel.add(capListPane);

		JPanel prodPanel = new JPanel();
		prodPanel.setBorder(new TitledBorder(null, "Product",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(prodPanel);
		prodPanel.setLayout(new BoxLayout(prodPanel, BoxLayout.Y_AXIS));

		JButton addProd = new JButton("Create Product");
		addProd.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showProductDialog();
			}
		});
		prodPanel.add(addProd);

		JList productList = new JList();
		productList.setModel(products);
		productList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane productListPane = new JScrollPane(productList);
		prodPanel.add(productListPane);

		JPanel batchPanel = new JPanel();
		batchPanel.setBorder(new TitledBorder(null, "Batch",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(batchPanel);
		batchPanel.setLayout(new BoxLayout(batchPanel, BoxLayout.Y_AXIS));

		JButton addBatch = new JButton("Create Batch");
		addBatch.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showBatchDialog();
			}
		});
		batchPanel.add(addBatch);

		JList batchList = new JList();
		batchList.setModel(batches);
		batchList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane batchListPane = new JScrollPane(batchList);
		batchPanel.add(batchListPane);

		JPanel eqPanel = new JPanel();
		eqPanel.setBorder(new TitledBorder(null, "Equiplet",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(eqPanel);
		eqPanel.setLayout(new BoxLayout(eqPanel, BoxLayout.Y_AXIS));

		JButton addEquiplet = new JButton("Create Equiplet");
		addEquiplet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showEquipletDialog();
			}
		});
		eqPanel.add(addEquiplet);

		JList eqList = new JList();
		eqList.setModel(equiplets);
		eqList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane eqListPane = new JScrollPane(eqList);
		eqPanel.add(eqListPane);

		JPanel gridPanel = new JPanel();
		gridPanel.setBorder(new TitledBorder(null, "Grid",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(gridPanel);
		gridPanel.setLayout(new BoxLayout(gridPanel, BoxLayout.Y_AXIS));

		JButton addGrid = new JButton("Create Grid");
		addEquiplet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showGridDialog();
			}
		});
		gridPanel.add(addGrid);
		
		JList gridList = new JList();
		gridList.setModel(gridContent);
		gridList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane gridListPane = new JScrollPane(gridList);
		gridPanel.add(gridListPane);
		
		JPanel buttonPanel = new JPanel();
		getContentPane().add(buttonPanel);
		
		JButton exportButton = new JButton("Export Files");
		exportButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				writeBatchesCSV();
				writeCapabilitiesCSV();
				writeGridJSON();
				writeProductsCSV();
			}
		});
		buttonPanel.add(exportButton);
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
		/*
		if (win != null) {
			CreateCapabilityDialog dialog = new CreateCapabilityDialog(win, "Create new capability", ModalityType.APPLICATION_MODAL, equiplets);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this
			dialog.setVisible(true);

			if(dialog.isSuccess) {
				capabilities.addElement(dialog.getCapability());
				writeCapabilitiesCSV();
			}
		}*/
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
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void writeGridJSON() {
		
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
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
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
