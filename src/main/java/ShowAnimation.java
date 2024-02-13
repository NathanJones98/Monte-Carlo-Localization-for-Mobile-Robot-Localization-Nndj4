import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

/**
 * Original code from https://stackoverflow.com/a/38980029.
 * Adapted with the adjusted file path and file names.
 */
public class ShowAnimation {

    private JFrame frame;
    private JPanel pane;
    private Timer timer;
    private int nextImage = 0;
    private List<String> tempImages = new ArrayList<>();
    private List<String> images = new ArrayList<>();
    private Image img = null;
    
   
    public void createAndShowGui() {
    	// Set the file name String
    	File figDir = new File("savedFigs/");
    	if (figDir.isDirectory()){
    		tempImages = Arrays.asList(figDir.list());
    	} else {
    		System.out.println("Warning: You do not have savedFigs/ directory yet!");
    		System.exit(1);
    	}
    	if (tempImages.size() == 0){
    		System.out.println("Warning: You do not have any .png files saved in savedFigs/ directory yet!");
    		System.exit(1);
    	}
    	
    	for (int i = 0; i < tempImages.size(); i++) {
    		String fileName = tempImages.get(i);
    		if (fileName.contains(".png")) {
    			images.add(fileName);
    		}
    	}
    	Collections.sort(images);
    	
    	// Delay is set at 30 by default; you can change this if you like
        frame = new JFrame("Monte Carlo Localization");
        timer = new Timer(30, listener);
        pane = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                try {
                    img = ImageIO.read(new FileInputStream("savedFigs/" + images.get(nextImage)));
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                g.drawImage(img , 0, 0, 800, 800, this);
            }

            @Override
            public Dimension getPreferredSize() {
                return new Dimension(800, 800);
            }
        };

        timer.start();

        frame.getContentPane().add(pane);
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    ActionListener listener = new ActionListener() {   
        @Override
        public void actionPerformed(ActionEvent event) {
            nextImage = nextImage < images.size() - 1 ? nextImage + 1 : 0;
            pane.repaint();
        }
    };
    
    
    /**
     * Run this to animate the saved figures
     */
    public static void main (String args[]) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                new ShowAnimation().createAndShowGui();
            }
        });
    }
}