package MAS.simulation.graphics;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.JPanel;

public class EquipletView extends JPanel {

	private static final Map<String, Color> COLORS;
	static {
		Map<String, Color> map = new HashMap<>();
		map.put("idle", Color.LIGHT_GRAY);
		map.put("busy", Color.GREEN);
		map.put("error", Color.RED);
		map.put("error_ready", new Color(0x640000));
		map.put("error_finished", new Color(0x640000));
		map.put("error_repaired", new Color(0x006400));
		map.put("reconfig", Color.MAGENTA);
		COLORS = Collections.unmodifiableMap(map);
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String name;
	private String state;
	private List<String> services;
	private int queue;
	private int scheduled;
	private int executed;

	public EquipletView(String name, List<String> services, String state, int waiting, int scheduled, int executed) {
		this.name = name;
		this.state = state;
		this.services = services;
		this.queue = waiting;
		this.scheduled = scheduled;
		this.executed = executed;
	}

	public EquipletView(String name, String state, int waiting, int scheduled, int executed) {
		this.name = name;
		this.state = state;
		// TODO remove this services or do something
		this.services = new ArrayList<String>();
		this.queue = waiting;
		this.scheduled = scheduled;
		this.executed = executed;
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;

		setSize(Math.min(200, Math.max(40, getWidth())), Math.min(100, Math.max(40, getHeight())));

		// System.out.printf("DRAW EQUIPLET %s, %d : (%d, %d)", state, queue, getWidth(), getHeight());
		g2.setPaint(COLORS.get(state.toLowerCase()));
		g2.fillRect(0, 0, Math.max(40, getWidth()), Math.max(64, getHeight()));

		g2.setColor(Color.BLACK);
		g2.drawString(name, 5, 12);
		g2.drawString(String.format("Services: %s", Arrays.toString(services.toArray())), 5, 25);
		g2.drawString(String.format("State: %s", state), 5, 38);
		g2.drawString(String.format("Queue: %d", queue), 5, 51);
		g2.drawString(String.format("Scheduled: %d", scheduled), 5, 64);
		g2.drawString(String.format("Executed: %d", executed), 5, 77);

	}

	public void update(String state, int queue, int scheduled, int executed) {
		// System.out.printf("UPDATE EQUIPLET %s, %d : (%d, %d)", state, queue, getWidth(), getHeight());
		this.state = state;
		this.queue = queue;
		this.scheduled = scheduled;
		this.executed = executed;
		repaint();
	}

	public String toString() {
		return "View:" + name + "(" + getWidth() + "," + getHeight() + ")";
	}
}
