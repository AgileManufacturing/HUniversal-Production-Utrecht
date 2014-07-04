package simulation.graphics;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

import simulation.mas.Equiplet;
import simulation.util.Triple;

public class GridView extends JPanel {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private HashMap<String, EquipletView> components;

	public GridView() {
		super(new GridBagLayout());
		this.components = new HashMap<>();
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, getWidth(), getWidth());
		// System.out.println("DRAW GRID: " + Arrays.toString(components.toArray()));

		for (Entry<String, EquipletView> view : components.entrySet()) {
			view.getValue().revalidate();
		}
	}

	public void init(Collection<Equiplet> equiplets) {
		for (Equiplet equiplet : equiplets) {

			EquipletView component = new EquipletView(equiplet.getName(), equiplet.getServices(), equiplet.getState().toString(), equiplet.getWaiting(), equiplet.getScheduled(), equiplet.getExecuted());
			components.put(equiplet.getName(), component);
			component.setBackground(Color.WHITE);
			component.setBorder(BorderFactory.createLineBorder(Color.BLACK, 2));

			GridBagConstraints gbc = new GridBagConstraints();
			gbc.gridx = equiplet.getPosition().getX();
			gbc.gridy = equiplet.getPosition().getY();
			gbc.fill = GridBagConstraints.BOTH;
			gbc.anchor = GridBagConstraints.CENTER;
			gbc.insets = new Insets(10, 10, 10, 10);
			gbc.weighty = 1.0;
			gbc.weightx = 1.0;
			add(component, gbc);
		}
	}

	public void update(Collection<Equiplet> equiplets) {
		if (components.isEmpty()) {
			init(equiplets);
		} else {
			for (Equiplet equiplet : equiplets) {
				EquipletView view = components.get(equiplet.getName());
				view.update(equiplet.getState().toString(), equiplet.getWaiting(), equiplet.getScheduled(), equiplet.getExecuted());
			}
		}
	}

	public void update(List<Triple<String, List<String>, Triple<String, Integer, Integer>>> equipletStates) {
		// TODO static sim update
	}
}
