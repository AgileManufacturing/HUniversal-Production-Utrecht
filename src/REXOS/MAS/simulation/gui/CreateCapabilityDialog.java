package simulation.gui;

import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.JButton;

import simulation.data.Capability;
import sun.java2d.Disposer;

import java.awt.Window;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.Component;

public class CreateCapabilityDialog extends JDialog {
	private JTextField capName;
	private JTextField capDuration;
	private JButton create;
	private JButton cancel;
	
	public boolean isSuccess = false;
	
	public CreateCapabilityDialog(Window owner, String title, ModalityType applicationModal) {
		super(owner, title, applicationModal);
		
		JPanel panel = new JPanel();
		panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
		
		JLabel capNameLbl = new JLabel("Name");
		panel.add(capNameLbl);
		
		capName = new JTextField();
		panel.add(capName);
		capName.setColumns(10);
		
		JLabel capDurationLbl = new JLabel("Duration");
		panel.add(capDurationLbl);
		
		capDuration = new JTextField();
		panel.add(capDuration);
		capDuration.setColumns(10);
		
		create = new JButton("Create");
		create.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = true;
				dispose();
			}
		});
		panel.add(create);
		
		cancel = new JButton("Cancel");
		cancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = false;
				dispose();
			}
		});
		panel.add(cancel);
		
		getContentPane().add(panel);
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
