package simulation.gui;

import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import simulation.data.Capability;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;

public class CreateCapabilityDialog extends JDialog {
	/**
	 * 
	 */
	private static final long serialVersionUID = -4057365795259631191L;
	private JTextField capName;
	private JTextField capDuration;
	private JButton create;
	private JButton cancel;
	
	public boolean isSuccess = false;
	
	public CreateCapabilityDialog(Window owner, String title, ModalityType applicationModal) {
		super(owner, title, applicationModal);
		
		JPanel panel = new JPanel();
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{25, 100, 150, 0};
		gbl_panel.rowHeights = new int[]{25, 0, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 0.0, 0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		JLabel capNameLbl = new JLabel("Name");
		GridBagConstraints gbc_capNameLbl = new GridBagConstraints();
		gbc_capNameLbl.insets = new Insets(0, 0, 5, 5);
		gbc_capNameLbl.anchor = GridBagConstraints.WEST;
		gbc_capNameLbl.gridx = 1;
		gbc_capNameLbl.gridy = 1;
		panel.add(capNameLbl, gbc_capNameLbl);
		
		capName = new JTextField();
		GridBagConstraints gbc_capName = new GridBagConstraints();
		gbc_capName.insets = new Insets(0, 0, 5, 0);
		gbc_capName.gridx = 2;
		gbc_capName.gridy = 1;
		panel.add(capName, gbc_capName);
		capName.setColumns(10);
		
		JLabel capDurationLbl = new JLabel("Duration");
		GridBagConstraints gbc_capDurationLbl = new GridBagConstraints();
		gbc_capDurationLbl.insets = new Insets(0, 0, 5, 5);
		gbc_capDurationLbl.anchor = GridBagConstraints.WEST;
		gbc_capDurationLbl.gridx = 1;
		gbc_capDurationLbl.gridy = 2;
		panel.add(capDurationLbl, gbc_capDurationLbl);
		
		capDuration = new JTextField();
		GridBagConstraints gbc_capDuration = new GridBagConstraints();
		gbc_capDuration.insets = new Insets(0, 0, 5, 0);
		gbc_capDuration.gridx = 2;
		gbc_capDuration.gridy = 2;
		panel.add(capDuration, gbc_capDuration);
		capDuration.setColumns(10);
		
		create = new JButton("Create");
		create.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				boolean pass = true;
				
				if(capName.getText().equals("")) {
					JOptionPane.showMessageDialog(null, "You must enter a name");
					pass = false;
				}
				
				try {
					Integer.parseInt(capDuration.getText());
				} catch (NumberFormatException e) {
					JOptionPane.showMessageDialog(null, "Wrong format for Duration. Must be a number");
					pass = false;
				}
				
				if(pass) {
					isSuccess = true;
					dispose();
				}
			}
		});
		GridBagConstraints gbc_create = new GridBagConstraints();
		gbc_create.insets = new Insets(0, 0, 0, 5);
		gbc_create.gridx = 1;
		gbc_create.gridy = 3;
		panel.add(create, gbc_create);
		
		getContentPane().add(panel);
		
		cancel = new JButton("Cancel");
		cancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = false;
				dispose();
			}
		});
		GridBagConstraints gbc_cancel = new GridBagConstraints();
		gbc_cancel.gridx = 2;
		gbc_cancel.gridy = 3;
		panel.add(cancel, gbc_cancel);
	}
	
	public String getCapabilityName() {
		return capName.getText();
	}
	
	public int getCapabilityDuration() {
		return Integer.parseInt(capDuration.getText());
	}
	
	public Capability getCapability() {
		return new Capability(getCapabilityName(), getCapabilityDuration());
	}
	
}
