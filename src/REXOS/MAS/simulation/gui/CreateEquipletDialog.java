package simulation.gui;

import java.awt.Dialog.ModalityType;
import java.awt.Window;

import javax.swing.DefaultListModel;
import javax.swing.JDialog;
import javax.swing.JPanel;

import simulation.data.BatchDescription;
import simulation.data.Capability;
import simulation.data.EquipletDescription;

import java.awt.BorderLayout;
import java.awt.GridBagLayout;

public class CreateEquipletDialog extends JDialog {

	public boolean isSuccess;

	public CreateEquipletDialog(Window win, String title, ModalityType applicationModal, DefaultListModel<Capability> capabilities, DefaultListModel<BatchDescription> batches) {
		super(win, title, applicationModal);
		
		JPanel panel = new JPanel();
		getContentPane().add(panel, BorderLayout.CENTER);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{0};
		gbl_panel.rowHeights = new int[]{0};
		gbl_panel.columnWeights = new double[]{Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
	}

	public EquipletDescription getEquiplet() {
		// TODO Auto-generated method stub
		return null;
	}

}
