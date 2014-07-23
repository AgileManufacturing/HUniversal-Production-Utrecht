package simulation.graphics;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

import simulation.mas.equiplet.Equiplet;
import simulation.mas.equiplet.EquipletAgent;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

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

			EquipletView component = new EquipletView(equiplet.getEquipletName(), equiplet.getServices(), equiplet.getEquipletState().toString(), equiplet.getWaiting(),
					equiplet.getScheduled(), equiplet.getExecuted());
			components.put(equiplet.getEquipletName(), component);
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
				EquipletView view = components.get(equiplet.getEquipletName());
				view.update(equiplet.getEquipletState().toString(), equiplet.getWaiting(), equiplet.getScheduled(), equiplet.getExecuted());
			}
		}
	}

	/**
	 * Update grid view Containing a box of the equiplet view
	 * 
	 * @param equiplets
	 */
	public void update(Map<String, Pair<Position, Tuple<String, Integer, Integer, Integer>>> equiplets) {
		for (Entry<String, Pair<Position, Tuple<String, Integer, Integer, Integer>>> entry : equiplets.entrySet()) {
			String name = entry.getKey();
			Position position = entry.getValue().first;
			String state = entry.getValue().second.first;
			int waiting = entry.getValue().second.second;
			int scheduled = entry.getValue().second.third;
			int executed = entry.getValue().second.fourth;

			EquipletView component = new EquipletView(name, state, waiting, scheduled, executed);
			// components.put(name, component);
			component.setBackground(Color.WHITE);
			component.setBorder(BorderFactory.createLineBorder(Color.BLACK, 2));

			GridBagConstraints gbc = new GridBagConstraints();
			gbc.gridx = position.getX();
			gbc.gridy = position.getY();
			gbc.fill = GridBagConstraints.BOTH;
			gbc.anchor = GridBagConstraints.CENTER;
			gbc.insets = new Insets(10, 10, 10, 10);
			gbc.weighty = 1.0;
			gbc.weightx = 1.0;
			add(component, gbc);
		}
	}

	public void update(List<Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>> equipletStates) {
		removeAll();
		for (Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> equiplet : equipletStates) {
			String name = equiplet.first;
			Position position = equiplet.second;

			String state = equiplet.fourth.first;
			int waiting = equiplet.fourth.second;
			int scheduled = equiplet.fourth.third;
			int executed = equiplet.fourth.fourth;

			EquipletView component = new EquipletView(name, state, waiting, scheduled, executed);
			// components.put(name, component);
			component.setBackground(Color.WHITE);
			component.setBorder(BorderFactory.createLineBorder(Color.BLACK, 2));

			GridBagConstraints gbc = new GridBagConstraints();
			gbc.gridx = position.getX();
			gbc.gridy = position.getY();
			gbc.fill = GridBagConstraints.BOTH;
			gbc.anchor = GridBagConstraints.CENTER;
			gbc.insets = new Insets(10, 10, 10, 10);
			gbc.weighty = 1.0;
			gbc.weightx = 1.0;
			add(component, gbc);
		}
		revalidate();
	}
}
