package simulation.gui;

import java.awt.BorderLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;

import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

import simulation.data.EquipletDescription;

public class ModifyRowDialog extends JDialog {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6553580636018441528L;
	private DefaultListModel<EquipletDescription> equiplets;
	private DefaultListModel<EquipletDescription> rowContents = new DefaultListModel<EquipletDescription>();
	private JList<EquipletDescription> eqList = new JList<EquipletDescription>();
	private JList<EquipletDescription> rowList = new JList<EquipletDescription>();
	
	public boolean isSuccess = false;
	
	public ModifyRowDialog(Window win, String title, 	ModalityType applicationModal, DefaultListModel<EquipletDescription> eqs, GridRowDescription row) {
		super(win, title, applicationModal);
		
		equiplets = eqs;
		LinkedList<EquipletDescription> eqsInRow = row.getEquiplets();
		for(int i = 0; i < eqsInRow.size(); i++) {
			rowContents.addElement(eqsInRow.get(i));
		}
		
		JPanel panel = new JPanel();
		getContentPane().add(panel, BorderLayout.CENTER);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.rowHeights = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		eqList.setModel(equiplets);
		eqList.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
		JScrollPane eqListPane = new JScrollPane(eqList);
		GridBagConstraints gbc_eqListPane = new GridBagConstraints();
		gbc_eqListPane.insets = new Insets(0, 0, 5, 5);
		gbc_eqListPane.gridheight = 5;
		gbc_eqListPane.gridwidth = 3;
		gbc_eqListPane.fill = GridBagConstraints.BOTH;
		gbc_eqListPane.gridx = 1;
		gbc_eqListPane.gridy = 1;
		panel.add(eqListPane, gbc_eqListPane);
		
		rowList.setModel(rowContents);
		rowList.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
		JScrollPane rowListPane = new JScrollPane(rowList);
		GridBagConstraints gbc_rowListPane = new GridBagConstraints();
		gbc_rowListPane.insets = new Insets(0, 0, 5, 5);
		gbc_rowListPane.gridheight = 5;
		gbc_rowListPane.gridwidth = 3;
		gbc_rowListPane.fill = GridBagConstraints.BOTH;
		gbc_rowListPane.gridx = 7;
		gbc_rowListPane.gridy = 1;
		panel.add(rowListPane, gbc_rowListPane);
		
		JButton btnModify = new JButton("Modify");
		btnModify.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = true;
				dispose();
			}
		});
		GridBagConstraints gbc_btnModify = new GridBagConstraints();
		gbc_btnModify.gridwidth = 2;
		gbc_btnModify.insets = new Insets(0, 0, 5, 5);
		gbc_btnModify.gridx = 1;
		gbc_btnModify.gridy = 9;
		panel.add(btnModify, gbc_btnModify);
		
		JButton btnAddToRow = new JButton(">");
		btnAddToRow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				// Add Equiplet to Row (and eventually remove from equiplets)
				if(rowList.getSelectedIndex() != -1) {
					rowContents.addElement(equiplets.elementAt(eqList.getSelectedIndex())); // Change to multiple indexes
					equiplets.remove(eqList.getSelectedIndex());
				}
			}
		});
		GridBagConstraints gbc_btnAddToRow = new GridBagConstraints();
		gbc_btnAddToRow.insets = new Insets(0, 0, 5, 5);
		gbc_btnAddToRow.gridx = 5;
		gbc_btnAddToRow.gridy = 2;
		panel.add(btnAddToRow, gbc_btnAddToRow);
		
		JButton btnRemoveFromRow = new JButton("<");
		btnRemoveFromRow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				// Remove Equiplet from Row
				if(rowList.getSelectedIndex() != -1) {
					equiplets.addElement(rowContents.elementAt(rowList.getSelectedIndex())); // Change to multiple indexes
					rowContents.remove(rowList.getSelectedIndex());
				}
			}
		});
		GridBagConstraints gbc_btnRemoveFromRow = new GridBagConstraints();
		gbc_btnRemoveFromRow.insets = new Insets(0, 0, 5, 5);
		gbc_btnRemoveFromRow.gridx = 5;
		gbc_btnRemoveFromRow.gridy = 4;
		panel.add(btnRemoveFromRow, gbc_btnRemoveFromRow);
		
		JButton btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				isSuccess = false;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCancel = new GridBagConstraints();
		gbc_btnCancel.gridwidth = 2;
		gbc_btnCancel.insets = new Insets(0, 0, 5, 5);
		gbc_btnCancel.gridx = 8;
		gbc_btnCancel.gridy = 9;
		panel.add(btnCancel, gbc_btnCancel);
	}
	
	public GridRowDescription getRow() {
		return new GridRowDescription(rowContents);
	}
}
