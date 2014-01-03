package simulation.gui;

import java.awt.BorderLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;

import simulation.data.BatchDescription;
import simulation.data.Capability;
import simulation.data.EquipletDescription;

public class CreateEquipletDialog extends JDialog {
	/**
	 * 
	 */
	private static final long serialVersionUID = -1239449918048720404L;
	private DefaultListModel<Capability> capabilities;
	private DefaultListModel<BatchDescription> batches;
	public boolean isSuccess;
	private JTextField nameText;
	private JList<Capability> capList = new JList<Capability>();
	private JList<BatchDescription> batchList = new JList<BatchDescription>();

	public CreateEquipletDialog(Window win, String title, ModalityType applicationModal, DefaultListModel<Capability> caps, DefaultListModel<BatchDescription> batchModel) {
		super(win, title, applicationModal);
		
		batches = batchModel;
		capabilities = caps;
		
		JPanel panel = new JPanel();
		getContentPane().add(panel, BorderLayout.CENTER);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.rowHeights = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		nameText = new JTextField();
		nameText.setUI(new WatermarkTextFieldUI("Name"));
		GridBagConstraints gbc_nameText = new GridBagConstraints();
		gbc_nameText.gridwidth = 3;
		gbc_nameText.insets = new Insets(0, 0, 5, 5);
		gbc_nameText.fill = GridBagConstraints.HORIZONTAL;
		gbc_nameText.gridx = 1;
		gbc_nameText.gridy = 1;
		panel.add(nameText, gbc_nameText);
		nameText.setColumns(10);
		
		JLabel lblReserveforlabel = new JLabel("Reserve for batch:");
		GridBagConstraints gbc_lblReserveforlabel = new GridBagConstraints();
		gbc_lblReserveforlabel.gridwidth = 4;
		gbc_lblReserveforlabel.insets = new Insets(0, 0, 5, 5);
		gbc_lblReserveforlabel.gridx = 7;
		gbc_lblReserveforlabel.gridy = 1;
		panel.add(lblReserveforlabel, gbc_lblReserveforlabel);
		
		JScrollPane capListPane = new JScrollPane(capList);
		capList.setModel(capabilities);
		capList.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
		GridBagConstraints gbc_capListPane = new GridBagConstraints();
		gbc_capListPane.gridheight = 5;
		gbc_capListPane.gridwidth = 4;
		gbc_capListPane.insets = new Insets(0, 0, 5, 5);
		gbc_capListPane.fill = GridBagConstraints.BOTH;
		gbc_capListPane.gridx = 1;
		gbc_capListPane.gridy = 2;
		panel.add(capListPane, gbc_capListPane);
		
		JScrollPane batchListPane = new JScrollPane(batchList);
		batchList.setModel(batches);
		batchList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		GridBagConstraints gbc_batchListPane = new GridBagConstraints();
		gbc_batchListPane.gridheight = 5;
		gbc_batchListPane.gridwidth = 5;
		gbc_batchListPane.insets = new Insets(0, 0, 5, 5);
		gbc_batchListPane.fill = GridBagConstraints.BOTH;
		gbc_batchListPane.gridx = 7;
		gbc_batchListPane.gridy = 2;
		panel.add(batchListPane, gbc_batchListPane);
		
		JButton btnCreate = new JButton("Create");
		btnCreate.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				boolean pass = true;
				
				if(nameText.getText().equals("")) {
					JOptionPane.showMessageDialog(null, "You must enter a name");
					pass = false;
				}
				
				if(capabilities.size() == 0) {
					JOptionPane.showMessageDialog(null, "An Equiplet must have at least one capability");
					pass = false;
				}
				
				if(pass) {
					isSuccess = true;
					dispose();
				}
			}
		});
		GridBagConstraints gbc_btnCreate = new GridBagConstraints();
		gbc_btnCreate.insets = new Insets(0, 0, 0, 5);
		gbc_btnCreate.gridx = 3;
		gbc_btnCreate.gridy = 8;
		panel.add(btnCreate, gbc_btnCreate);
		
		JButton btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = false;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCancel = new GridBagConstraints();
		gbc_btnCancel.insets = new Insets(0, 0, 0, 5);
		gbc_btnCancel.gridx = 9;
		gbc_btnCancel.gridy = 8;
		panel.add(btnCancel, gbc_btnCancel);
	}

	public EquipletDescription getEquiplet() {
		int[] indices = capList.getSelectedIndices();
		Capability[] caps = new Capability[indices.length];
		
		for(int i = 0; i < indices.length; i++) {
			caps[i] = capabilities.elementAt(indices[i]);
		}
		
		if(batchList.getSelectedIndex() == -1) {
			return new EquipletDescription(nameText.getText(), caps);
		} else {
			return new EquipletDescription(nameText.getText(), caps, batchList.getSelectedIndex());
		}
	}

}
