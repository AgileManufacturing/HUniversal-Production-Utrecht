package simulation.gui;

import java.awt.BorderLayout;
import java.awt.GridBagLayout;
import java.awt.Window;

import javax.swing.DefaultListModel;
import javax.swing.JDialog;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

import simulation.data.EquipletDescription;

import javax.swing.JList;

import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;

import javax.swing.JButton;

public class CreateGridDialog extends JDialog {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6678432279134231602L;
	private DefaultListModel<EquipletDescription> equiplets = new DefaultListModel<EquipletDescription>();
	private DefaultListModel<GridRowDescription> rows = new DefaultListModel<GridRowDescription>();
	private JList<GridRowDescription> rowList = new JList<GridRowDescription>();
	private Window window;
	
	public boolean isSuccess = false;
	
	public CreateGridDialog(Window win, String title, ModalityType applicationModal, DefaultListModel<EquipletDescription> eqs) {
		super(win, title, applicationModal);
		
		window = win;
		
		for(int i = 0; i < eqs.size(); i++) {
			equiplets.addElement(eqs.elementAt(i));
		}
		
		JPanel panel = new JPanel();
		getContentPane().add(panel, BorderLayout.CENTER);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.rowHeights = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		rowList.setModel(rows);
		rowList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		JScrollPane rowListPane = new JScrollPane(rowList);
		GridBagConstraints gbc_rowListPane = new GridBagConstraints();
		gbc_rowListPane.insets = new Insets(0, 0, 5, 5);
		gbc_rowListPane.gridheight = 5;
		gbc_rowListPane.gridwidth = 7;
		gbc_rowListPane.fill = GridBagConstraints.BOTH;
		gbc_rowListPane.gridx = 1;
		gbc_rowListPane.gridy = 1;
		panel.add(rowListPane, gbc_rowListPane);
		
		JButton btnCreate = new JButton("Create");
		btnCreate.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = true;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCreate = new GridBagConstraints();
		gbc_btnCreate.gridwidth = 2;
		gbc_btnCreate.insets = new Insets(0, 0, 5, 5);
		gbc_btnCreate.gridx = 1;
		gbc_btnCreate.gridy = 9;
		panel.add(btnCreate, gbc_btnCreate);
		
		JButton btnAddRow = new JButton("Add Row");
		btnAddRow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				rows.addElement(new GridRowDescription());
			}
		});
		GridBagConstraints gbc_btnAddRow = new GridBagConstraints();
		gbc_btnAddRow.gridwidth = 2;
		gbc_btnAddRow.insets = new Insets(0, 0, 5, 5);
		gbc_btnAddRow.gridx = 1;
		gbc_btnAddRow.gridy = 7;
		panel.add(btnAddRow, gbc_btnAddRow);
		
		JButton btnModifyRow = new JButton("Modify Row");
		btnModifyRow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				if(rowList.getSelectedIndex() != -1) {
					GridRowDescription row = rows.elementAt(rowList.getSelectedIndex());
					ModifyRowDialog dialog = new ModifyRowDialog(window, "Modify Row", ModalityType.APPLICATION_MODAL, equiplets, row);
					dialog.pack();
					dialog.setLocationRelativeTo(null); // fix this
					dialog.setVisible(true);
					
					if(dialog.isSuccess) {
						rows.set(rowList.getSelectedIndex(), dialog.getRow());
					}
				}
			}
		});
		GridBagConstraints gbc_btnModifyRow = new GridBagConstraints();
		gbc_btnModifyRow.insets = new Insets(0, 0, 5, 5);
		gbc_btnModifyRow.gridx = 4;
		gbc_btnModifyRow.gridy = 7;
		panel.add(btnModifyRow, gbc_btnModifyRow);
		
		JButton btnRemoveRow = new JButton("Remove Row");
		btnRemoveRow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(rowList.getSelectedIndex() != -1) {
					GridRowDescription row = rows.remove(rowList.getSelectedIndex());
					LinkedList<EquipletDescription> eqs = row.getEquiplets();
					for(int i = 0; i < eqs.size(); i++) {
						equiplets.addElement(eqs.get(i));
					}
				}
			}
		});
		GridBagConstraints gbc_btnRemoveRow = new GridBagConstraints();
		gbc_btnRemoveRow.gridwidth = 2;
		gbc_btnRemoveRow.insets = new Insets(0, 0, 5, 5);
		gbc_btnRemoveRow.gridx = 6;
		gbc_btnRemoveRow.gridy = 7;
		panel.add(btnRemoveRow, gbc_btnRemoveRow);
		
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
		gbc_btnCancel.gridx = 6;
		gbc_btnCancel.gridy = 9;
		panel.add(btnCancel, gbc_btnCancel);
	}
	
	public EquipletDescription[][] getGrid() {
		EquipletDescription[][] grid = new EquipletDescription[rows.size()][];
		
		for(int i = 0; i < rows.size(); i++) {
			LinkedList<EquipletDescription> equiplets = rows.elementAt(i).getEquiplets();
			grid[i] = new EquipletDescription[equiplets.size()];
			
			for(int j = 0; j < equiplets.size(); j++) {
				grid[i][j] = equiplets.get(j); 
			}
		}
		
		return grid;
	}
}
