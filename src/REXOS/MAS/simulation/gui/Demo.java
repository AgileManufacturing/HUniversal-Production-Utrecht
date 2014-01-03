package simulation.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.image.MemoryImageSource;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.SwingWorker;

import simulation.gui.Visualisation.DrawGrid;

public class Demo extends javax.swing.JPanel {
    private DrawGrid eq = new DrawGrid(50, 50);
    public Demo() {
        new Worker().execute();
    }
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        eq.draw(g);
        //g.drawImage(src, 0, 0, this);
    }
    private class Worker extends SwingWorker<Void, Image>{
        private final Color[] colors = { Color.red, Color.green, Color.blue };
        protected void process(List<Image> chunks){
            for (Image bufferedImage : chunks){
                //src = bufferedImage;
                repaint();
            }
        }
        protected Void doInBackground() throws Exception{
            int frames = 0;
            int[] mem = new int[1024 * 768];
            long start = System.currentTimeMillis();
            long end = start + 15000;
            long last = start;
            while (last < end){
                int col = colors[frames % colors.length].getRGB();
                for (int y = 0; y < 768; y++)
                    for (int x = 0; x < 1024; x++)
                        mem[x + y * 1024] = col;
                Image img = createImage(new MemoryImageSource(1024, 768, mem, 0, 1024));
                BufferedImage bi = new BufferedImage(1024, 768, BufferedImage.TYPE_INT_ARGB);
                Graphics2D g2 = bi.createGraphics();
                g2.drawImage(img, 0, 0, null);
                g2.dispose();
                publish(bi);
                last = System.currentTimeMillis();
                frames++;
            }
            System.err.println("Frames = " + frames + ", fps = " + ((double) frames / (last - start) * 1000));
            return null;
        }
    }
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run(){
                JFrame jf = new JFrame();
                jf.getContentPane().add(new Demo(), BorderLayout.CENTER);
                jf.setSize(1024, 768);
                jf.setVisible(true);
            }
        });
    }
}