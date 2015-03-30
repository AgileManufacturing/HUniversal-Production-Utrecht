package MAS.simulation.graphics;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.BoxLayout;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTree;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.TreeSelectionModel;

import MAS.product.Product;

public class ProductView extends JPanel implements TreeSelectionListener {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Map<String, Product> products;

	private JPanel scheduleView;
	private JTree tree;

	// Optionally play with line styles. Possible values are
	// "Angled" (the default), "Horizontal", and "None".
	private static boolean playWithLineStyle = false;
	private static String lineStyle = "Horizontal";

	public ProductView() {
		super(new GridLayout(1, 0));
		this.products = new HashMap<String, Product>();
	}

	public void update(Map<String, Product> products) {
		this.products = products;

		removeAll();

		// Create the nodes.
		DefaultMutableTreeNode top = new DefaultMutableTreeNode("Products");
		createNodes(top);

		// Create a tree that allows one selection at a time.
		tree = new JTree(top);
		tree.getSelectionModel().setSelectionMode(TreeSelectionModel.SINGLE_TREE_SELECTION);

		// Listen for when the selection changes.
		tree.addTreeSelectionListener(this);

		if (playWithLineStyle) {
			System.out.println("line style = " + lineStyle);
			tree.putClientProperty("JTree.lineStyle", lineStyle);
		}

		// Create the scroll pane and add the tree to it.
		JScrollPane treeView = new JScrollPane(tree);

		// Create the schedule panel.
		scheduleView = new JPanel();
		scheduleView.setLayout(new BoxLayout(scheduleView, BoxLayout.X_AXIS));

		JScrollPane htmlView = new JScrollPane(scheduleView);

		// Add the scroll panes to a split pane.
		JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT);
		splitPane.setTopComponent(treeView);
		splitPane.setBottomComponent(htmlView);

		Dimension minimumSize = new Dimension(100, 250);
		htmlView.setMinimumSize(minimumSize);
		treeView.setMinimumSize(minimumSize);
		splitPane.setDividerLocation(100); // XXX: ignored in some releases
											// of Swing. bug 4101306
		// workaround for bug 4101306:
		// treeView.setPreferredSize(new Dimension(100, 100));

		splitPane.setPreferredSize(new Dimension(500, 300));

		// Add the split pane to this panel.
		add(splitPane);
	}

	/** Required by TreeSelectionListener interface. */
	public void valueChanged(TreeSelectionEvent e) {
		DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();

		if (node == null) {
			return;
		}

		Object nodeInfo = node.getUserObject();
		if (node.isLeaf()) {
			nodeInfo = node.getParent();
		}

		if (!node.isLeaf()) {
			Product product = products.get(nodeInfo.toString());
			scheduleView.removeAll();
			List<Product> list = new ArrayList<Product>();
			list.add(product);
			// scheduleView.add(GanttChart.createChartProducts(list));
			scheduleView.revalidate();
			throw new RuntimeException("not supported");
		}
	}

	private void createNodes(DefaultMutableTreeNode top) {
		for (Entry<String, Product> entry : products.entrySet()) {
			Product product = entry.getValue();
			DefaultMutableTreeNode category = new DefaultMutableTreeNode(product.getProductName());
			top.add(category);

			category.add(new DefaultMutableTreeNode(product.getCreated()));
			category.add(new DefaultMutableTreeNode(product.getProductState()));
			category.add(new DefaultMutableTreeNode(product.getPosition()));
			category.add(new DefaultMutableTreeNode(product.getExecutingStep()));
			category.add(new DefaultMutableTreeNode(product.getNextEquiplet()));
			category.add(new DefaultMutableTreeNode(product.getProductionPath()));
		}
	}

	public void updates(Map<String, Product> products) {
		this.products = products;

		removeAll();

		JTree tree = new JTree();

		JScrollPane scrollPane = new JScrollPane(tree);
		add(scrollPane, BorderLayout.CENTER);

		revalidate();
	}
}
