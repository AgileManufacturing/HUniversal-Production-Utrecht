package simulation.gui;

import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.JButton;

public class CreateCapabilityDialog extends JPanel {
	private JTextField capName;
	private JTextField capDuration;
	private JButton create;
	private JButton cancel;
	
	public CreateCapabilityDialog() {
		
		JLabel capNameLbl = new JLabel("Name");
		add(capNameLbl);
		
		capName = new JTextField();
		add(capName);
		capName.setColumns(10);
		
		JLabel capDurationLbl = new JLabel("Duration");
		add(capDurationLbl);
		
		capDuration = new JTextField();
		add(capDuration);
		capDuration.setColumns(10);
		
		create = new JButton("Create");
		add(create);
		
		cancel = new JButton("Cancel");
		add(cancel);
	}
	
	public String getCapabilityName() {
		return capName.getText();
	}
	
	public String getCapabilityDuration() {
		return capDuration.getText();
	}
	
}
