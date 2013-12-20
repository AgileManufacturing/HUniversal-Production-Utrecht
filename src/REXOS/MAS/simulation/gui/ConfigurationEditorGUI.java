package simulation.gui;

import jade.util.leap.Iterator;

import java.awt.Dialog.ModalityType;
import java.awt.FlowLayout;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

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

public class ConfigurationEditorGUI extends JFrame {
	Window win;
	
	LinkedHashMap<String, String> capabilities = new LinkedHashMap<String, String>();
	DefaultListModel<String> capNames = new DefaultListModel<String>();

	LinkedHashMap<String, LinkedHashMap<String, String>> batches = new LinkedHashMap<String, LinkedHashMap<String, String>>();
	DefaultListModel<String> batchNames = new DefaultListModel<String>();

	LinkedHashMap<String, LinkedHashMap<String, String>> products = new LinkedHashMap<String, LinkedHashMap<String, String>>();
	DefaultListModel<String> productNames = new DefaultListModel<String>();

	LinkedHashMap<String, LinkedHashMap<String, String>> equiplets = new LinkedHashMap<String, LinkedHashMap<String, String>>();
	DefaultListModel<String> equipletNames = new DefaultListModel<String>();

	JPanel capPanel = new JPanel();

	public ConfigurationEditorGUI() {
		setTitle("Configuration Editor");
		setSize(417, 308);
		setLocationRelativeTo(null);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		getContentPane().setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		capPanel.setBorder(new TitledBorder(null, "Capability",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(capPanel);
		win = SwingUtilities.getWindowAncestor(capPanel);
		capPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		JButton addCap = new JButton("Create Capability");
		addCap.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showCapabilityDialog();
			}
		});
		capPanel.add(addCap);

		JList capList = new JList();
		capList.setModel(capNames);
		capList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane capListPane = new JScrollPane(capList);
		capPanel.add(capListPane);

		JPanel prodPanel = new JPanel();
		prodPanel.setBorder(new TitledBorder(null, "Product",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(prodPanel);
		prodPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		JButton addProd = new JButton("Create Product");
		addProd.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showProductDialog();
			}
		});
		prodPanel.add(addProd);

		JList productList = new JList();
		productList.setModel(productNames);
		productList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane productListPane = new JScrollPane(productList);
		prodPanel.add(productListPane);

		JPanel batchPanel = new JPanel();
		batchPanel.setBorder(new TitledBorder(null, "Batch",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(batchPanel);
		batchPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		JButton addBatch = new JButton("Create Batch");
		addBatch.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showBatchDialog();
			}
		});
		batchPanel.add(addBatch);

		JList batchList = new JList();
		batchList.setModel(batchNames);
		batchList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane batchListPane = new JScrollPane(batchList);
		batchPanel.add(batchListPane);

		JPanel eqPanel = new JPanel();
		eqPanel.setBorder(new TitledBorder(null, "Equiplet",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(eqPanel);
		eqPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		JButton addEquiplet = new JButton("Create Equiplet");
		addEquiplet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showEquipletDialog();
			}
		});
		eqPanel.add(addEquiplet);

		JList eqList = new JList();
		eqList.setModel(equipletNames);
		eqList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane eqListPane = new JScrollPane(eqList);
		eqPanel.add(eqListPane);

		JPanel gridPanel = new JPanel();
		gridPanel.setBorder(new TitledBorder(null, "Grid",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		getContentPane().add(gridPanel);
		gridPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));

		JButton addGrid = new JButton("Create Grid");
		addEquiplet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				showGridDialog();
			}
		});
		gridPanel.add(addGrid);

		JList gridList = new JList();
		gridList.setModel(equipletNames);
		gridList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

		JScrollPane gridListPane = new JScrollPane(gridList);
		gridPanel.add(gridListPane);
	}

	protected void showCapabilityDialog() {
		if (win != null) {
			JDialog dialog = new JDialog(win, "Create new capability",
					ModalityType.APPLICATION_MODAL);
			CreateCapabilityDialog capDialog = new CreateCapabilityDialog();
			dialog.getContentPane().add(capDialog);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this

			dialog.setVisible(true);

			// add new capability to the list
			capabilities.put(capDialog.getCapabilityName(),
					capDialog.getCapabilityDuration());
			capNames.addElement(capDialog.getCapabilityName());
		}
	}

	protected void showBatchDialog() {
		if (win != null) {
			JDialog dialog = new JDialog(win, "Create new batch",
					ModalityType.APPLICATION_MODAL);
			CreateBatchDialog batchDialog = new CreateBatchDialog();
			dialog.getContentPane().add(batchDialog);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this

			dialog.setVisible(true);
			
			writeCapabilitiesCSV();
			
//			batches.put(batchDialog.getBatchName(), batchDialog.getValues());
//			batchNames.addElement(batchDialog.getBatchName());
		}
	}

	protected void showEquipletDialog() {
		if (win != null) {
			JDialog dialog = new JDialog(win, "Create new equiplet",
					ModalityType.APPLICATION_MODAL);
			CreateEquipletDialog eqDialog = new CreateEquipletDialog();
			dialog.getContentPane().add(eqDialog);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this

			dialog.setVisible(true);
			
//			equiplets.put(eqDialog.getEquipletName(), eqDialog.getValues());
//			equipletNames.addElement(eqDialog.getEquipletName());
		}
	}

	protected void showProductDialog() {
		if (win != null) {
			JDialog dialog = new JDialog(win, "Create new product",
					ModalityType.APPLICATION_MODAL);
			CreateProductDialog productDialog = new CreateProductDialog();
			dialog.getContentPane().add(productDialog);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this

			dialog.setVisible(true);
			
//			products.put(productDialog.getProductName(), productDialog.getValues());
//			productNames.addElement(productDialog.getProductName());
		}
	}
	
	protected void showGridDialog() {
		if (win != null) {
			JDialog dialog = new JDialog(win, "Create new grid",
					ModalityType.APPLICATION_MODAL);
			CreateGridDialog gridDialog = new CreateGridDialog();
			dialog.getContentPane().add(gridDialog);
			dialog.pack();
			dialog.setLocationRelativeTo(null); // fix this

			dialog.setVisible(true);
		}
	}
	
	private void writeProductsCSV() {
		
	}
	
	private void writeGridJSON() {
		
	}
	
	private void writeCapabilitiesCSV() {
		try {
			PrintWriter writer = new PrintWriter("capabilities.csv", "UTF-8");
			
			java.util.Iterator<Entry<String, String>> it = capabilities.entrySet().iterator();
			int id = 0;
			while(it.hasNext()) {
				Entry<String, String> entry = it.next();
				writer.write(id + "," + entry.getKey() + "," + entry.getValue() + "\r\n");
				id++;
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
