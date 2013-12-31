package simulation.gui;

import java.awt.Dialog.ModalityType;
import java.awt.Window;

import javax.swing.DefaultListModel;
import javax.swing.JDialog;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

import simulation.data.BatchDescription;
import simulation.data.Capability;
import simulation.data.ProductDescription;
import simulation.data.ProductStep;

import java.awt.BorderLayout;

import javax.swing.BoxLayout;
import javax.swing.JTextField;

import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;

import javax.swing.JButton;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

public class CreateBatchDialog extends JDialog {

	public boolean isSuccess = false;
	private JTextField numRuns;
	private JList prodList = new JList();
	private DefaultListModel<ProductDescription> products;

	public CreateBatchDialog(Window win, String title, 	ModalityType applicationModal, DefaultListModel<ProductDescription> products) {
		super(win, title, applicationModal);
		getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
		this.products = products;
		
		JPanel panel = new JPanel();
		getContentPane().add(panel);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{75, 75, 0};
		gbl_panel.rowHeights = new int[]{35, 100, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		numRuns = new JTextField();
		numRuns.setUI(new WatermarkTextFieldUI("# Products"));
		GridBagConstraints gbc_numRuns = new GridBagConstraints();
		gbc_numRuns.gridwidth = 2;
		gbc_numRuns.fill = GridBagConstraints.HORIZONTAL;
		gbc_numRuns.insets = new Insets(5, 5, 5, 0);
		gbc_numRuns.gridx = 0;
		gbc_numRuns.gridy = 0;
		panel.add(numRuns, gbc_numRuns);
		numRuns.setColumns(10);
				
		prodList.setModel(products);
		prodList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		JScrollPane prodListPane = new JScrollPane(prodList);
		GridBagConstraints gbc_prodListPane = new GridBagConstraints();
		gbc_prodListPane.gridwidth = 2;
		gbc_prodListPane.insets = new Insets(5, 5, 5, 0);
		gbc_prodListPane.fill = GridBagConstraints.BOTH;
		gbc_prodListPane.anchor = GridBagConstraints.NORTHWEST;
		gbc_prodListPane.gridx = 0;
		gbc_prodListPane.gridy = 1;
		panel.add(prodListPane, gbc_prodListPane);
		
		JButton btnCreate = new JButton("Create");
		btnCreate.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = true;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCreate = new GridBagConstraints();
		gbc_btnCreate.insets = new Insets(5, 5, 5, 5);
		gbc_btnCreate.gridx = 0;
		gbc_btnCreate.gridy = 2;
		panel.add(btnCreate, gbc_btnCreate);
		
		JButton btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				isSuccess = false;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCancel = new GridBagConstraints();
		gbc_btnCancel.insets = new Insets(5, 5, 5, 0);
		gbc_btnCancel.gridx = 1;
		gbc_btnCancel.gridy = 2;
		panel.add(btnCancel, gbc_btnCancel);
	}

	public BatchDescription getBatch() {
		int prodId = products.getElementAt(prodList.getSelectedIndex()).getId();
		BatchDescription batchDescription = new BatchDescription(prodId, Integer.parseInt(numRuns.getText()));
		return batchDescription;
	}

}
