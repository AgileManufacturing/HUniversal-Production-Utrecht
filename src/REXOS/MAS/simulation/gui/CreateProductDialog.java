package simulation.gui;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Window;

import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;

import simulation.Duration;
import simulation.data.Capability;
import simulation.data.ProductDescription;
import simulation.data.ProductStep;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

public class CreateProductDialog extends JDialog {
	private DefaultListModel<Capability> productSteps = new DefaultListModel<Capability>();
	private DefaultListModel<Capability> capabilities;
	public boolean isSuccess = false;
	private JTextField txtName;
	private JTextField txtDeadline;
	private JTextField txtCount;
	private JTextField txtInterval;
	private JList capList = new JList();
	private JList prodStepList = new JList();

	public CreateProductDialog(Window win, String title, ModalityType applicationModal, DefaultListModel<Capability> caps) {
		super(win, title, applicationModal);
		getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
		capabilities = caps;
		setPreferredSize(new Dimension(350, 200));
		
		JPanel panel = new JPanel();
		getContentPane().add(panel);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[] {75, 75, 75, 75, 75, 0};
		gbl_panel.rowHeights = new int[] {0, 0, 0, 0, 0, 0, 0, 0};
		gbl_panel.columnWeights = new double[]{0.0, 1.0, 1.0, 1.0, 0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		txtName = new JTextField();
		txtName.setUI(new WatermarkTextFieldUI("Name"));
		GridBagConstraints gbc_txtName = new GridBagConstraints();
		gbc_txtName.insets = new Insets(0, 0, 5, 5);
		gbc_txtName.fill = GridBagConstraints.HORIZONTAL;
		gbc_txtName.gridx = 1;
		gbc_txtName.gridy = 0;
		panel.add(txtName, gbc_txtName);
		txtName.setColumns(10);
		
		txtCount = new JTextField();
		txtCount.setUI(new WatermarkTextFieldUI("Count"));
		GridBagConstraints gbc_txtCount = new GridBagConstraints();
		gbc_txtCount.insets = new Insets(0, 0, 5, 5);
		gbc_txtCount.fill = GridBagConstraints.HORIZONTAL;
		gbc_txtCount.gridx = 3;
		gbc_txtCount.gridy = 0;
		panel.add(txtCount, gbc_txtCount);
		txtCount.setColumns(10);
		
		capList.setModel(caps);
		capList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		JScrollPane capListPane = new JScrollPane(capList);
		GridBagConstraints gbc_capListPane = new GridBagConstraints();
		gbc_capListPane.fill = GridBagConstraints.BOTH;
		gbc_capListPane.gridheight = 4;
		gbc_capListPane.gridwidth = 2;
		gbc_capListPane.insets = new Insets(2, 2, 5, 5);
		gbc_capListPane.gridx = 0;
		gbc_capListPane.gridy = 2;
		panel.add(capListPane, gbc_capListPane);
		
		txtDeadline = new JTextField();
		txtDeadline.setUI(new WatermarkTextFieldUI("Deadline"));
		GridBagConstraints gbc_txtDeadline = new GridBagConstraints();
		gbc_txtDeadline.insets = new Insets(0, 0, 5, 5);
		gbc_txtDeadline.fill = GridBagConstraints.HORIZONTAL;
		gbc_txtDeadline.gridx = 1;
		gbc_txtDeadline.gridy = 1;
		panel.add(txtDeadline, gbc_txtDeadline);
		txtDeadline.setColumns(10);
		
		txtInterval = new JTextField();
		txtInterval.setUI(new WatermarkTextFieldUI("Interval"));
		GridBagConstraints gbc_txtInterval = new GridBagConstraints();
		gbc_txtInterval.insets = new Insets(0, 0, 5, 5);
		gbc_txtInterval.fill = GridBagConstraints.HORIZONTAL;
		gbc_txtInterval.gridx = 3;
		gbc_txtInterval.gridy = 1;
		panel.add(txtInterval, gbc_txtInterval);
		txtInterval.setColumns(10);
		
		JButton addStep = new JButton("Queue");
		addStep.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				productSteps.addElement(capabilities.getElementAt(capList.getSelectedIndex()));
			}
		});
		GridBagConstraints gbc_addStep = new GridBagConstraints();
		gbc_addStep.insets = new Insets(0, 0, 5, 5);
		gbc_addStep.gridx = 2;
		gbc_addStep.gridy = 3;
		panel.add(addStep, gbc_addStep);
		
		JButton removeStep = new JButton("Remove");
		removeStep.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				prodStepList.remove(prodStepList.getSelectedIndex());
			}
		});
		GridBagConstraints gbc_removeStep = new GridBagConstraints();
		gbc_removeStep.insets = new Insets(0, 0, 5, 5);
		gbc_removeStep.gridx = 2;
		gbc_removeStep.gridy = 4;
		panel.add(removeStep, gbc_removeStep);
		
		prodStepList.setModel(productSteps);
		prodStepList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		JScrollPane prodStepListPane = new JScrollPane(prodStepList);
		GridBagConstraints gbc_prodStepListPane = new GridBagConstraints();
		gbc_prodStepListPane.insets = new Insets(0, 0, 5, 0);
		gbc_prodStepListPane.fill = GridBagConstraints.BOTH;
		gbc_prodStepListPane.gridy = 2;
		gbc_prodStepListPane.gridx = 3;
		gbc_prodStepListPane.gridwidth = 2;
		gbc_prodStepListPane.gridheight = 4;
		panel.add(prodStepListPane, gbc_prodStepListPane);
		
		JButton btnCreate = new JButton("Create");
		btnCreate.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				isSuccess = true;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCreate = new GridBagConstraints();
		gbc_btnCreate.insets = new Insets(0, 0, 0, 5);
		gbc_btnCreate.gridx = 1;
		gbc_btnCreate.gridy = 7;
		panel.add(btnCreate, gbc_btnCreate);
		
		JButton btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				isSuccess = false;
				dispose();
			}
		});
		GridBagConstraints gbc_btnCancel = new GridBagConstraints();
		gbc_btnCancel.insets = new Insets(0, 0, 0, 5);
		gbc_btnCancel.gridx = 3;
		gbc_btnCancel.gridy = 7;
		panel.add(btnCancel, gbc_btnCancel);
	}

	public ProductDescription getProduct() {
		ProductDescription productDescription;
		ArrayList<ProductStep> steps = new ArrayList<ProductStep>();
		
		for(int i = 0; i < prodStepList.getModel().getSize(); i++) {
			Capability capability = (Capability)prodStepList.getModel().getElementAt(i);
			steps.add(new ProductStep(capability));
		}
		
		System.out.println("Content: " + txtName.getText());
		
		if(txtName.getText().equals("")) {
			productDescription = new ProductDescription(Duration.parseDurationString(txtDeadline.getText()), Integer.parseInt(txtCount.getText()), Integer.parseInt(txtInterval.getText()), steps);
		} else {
			productDescription = new ProductDescription(txtName.getText(), Duration.parseDurationString(txtDeadline.getText()), Integer.parseInt(txtCount.getText()), Integer.parseInt(txtInterval.getText()), steps);
		}
		
		return productDescription;
	}

}
