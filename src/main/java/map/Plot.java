package map;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.io.File;

import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.*;

import mcl.Observation;
import mcl.Particle;

/**
 * A helper class used to visualize the map of Wean hall, particles, and laser readings.
 */
public class Plot extends JPanel{
	
	private GlobalMap globalMap;
	private JFrame frame;
	private BufferedImage mapImage;
	private Observation observation;
	private Particle robot;
	private List<Particle> particles;
	private int greyscale;
	private int size;
	private int figNum;
	private int drawMax;
	private boolean drawLoggedPose;
	
	
	public Plot(GlobalMap map, int drawMax, boolean drawLogPose) {
		this.drawLoggedPose = drawLogPose;
		this.drawMax = drawMax;
		this.globalMap = map;
		figNum = 0;
		
		// Create a directory root/savedFigs if not exists
		File figDir = new File("savedFigs");
		boolean dirCreated = figDir.mkdir();
		if (dirCreated) {
			System.out.println("savedFigs folder created...\n\n");
		}
		
		// Create a 800 x 800 grey scale map image
		size = 800;
		mapImage = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				double value = Double.parseDouble(globalMap.globalMapValues[i][j]);
				if (value < 0) {
					greyscale = 0;
				} else {
					greyscale = (int) (value * 255);
				}
				Color newColor = new Color(greyscale, greyscale, greyscale);	// either (0, 0, 0) or (255, 255, 255)
				mapImage.setRGB(i, j, newColor.getRGB());
			}
		}
		
		frame = new JFrame();
		frame.setSize(size, size);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(this);

		// Uncomment below if you want to know pixel values
//	    final JTextField text = new JTextField();
//	    frame.add(text, BorderLayout.SOUTH);
//	    frame.addMouseListener(new MouseListener() {
//	        public void mousePressed(MouseEvent me) { }
//	        public void mouseReleased(MouseEvent me) { }
//	        public void mouseEntered(MouseEvent me) { }
//	        public void mouseExited(MouseEvent me) { }
//	        public void mouseClicked(MouseEvent me) { 
//	          int x = me.getX();
//	          int y = me.getY();
//	          text.setText("X:" + x + " Y:" + y); 
//	        }
//	    });
	}
	
	/**
	 * Updates the plot such that the current list of particles are drawn with the particle having the largest weight
	 * showing up as a dot with a different color. Also, the laser readings spanning 180 degrees will also be plotted. 
	 * Note that when (drawLoggedPose == true), then we directly plot the pose recorded in the log file.
	 *  
	 * @param observation	an Observation object associated with a single time step
	 * @param particles		the current list of particles
	 */
	public void updatePlot(Observation observation, List<Particle> particles) {
		this.observation = observation;
		this.particles = particles;
		
		// particleList is sorted according to particle weights. Pick the most probable particle.
		robot = particles.get(particles.size() - 1);
		
		// Or, simply draw the current pose in log
		if (drawLoggedPose) {
			robot = robot.clone();
			robot.setPose(observation.robotPose);
		}
		repaint();
		saveFig();
		frame.setVisible(true);
	}
	
    /**
     * Saves the current plot into a .png file. 
     * <p>
     * Run the following ffmpeg command in project root directory to convert pngs to video:
     * ffmpeg -r 60 -f image2 -s 1920x1080 -i savedFigs/pic%04d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p animations/'filename'.mp4
     * (replace 'filename' with whatever you want to name the file)
     */
	private void saveFig() {
		BufferedImage image = new BufferedImage(800,  800, BufferedImage.TYPE_INT_RGB);
		Graphics2D graphics2D = image.createGraphics();
		frame.paint(graphics2D);
		try {
			ImageIO.write(image, "png", new File(String.format("savedFigs/pic%04d.png", figNum)));
			figNum += 1;
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		
		// Draw the global map
		g.drawImage(mapImage, 0, 0, null);
		
		// Draw particles
		g.setColor(new Color(50, 255, 150));
		int numTotalParticle = particles.size();
		int draw = Math.min(numTotalParticle, drawMax);
		for (int i = 0; i < draw; i++) {
			Particle p = particles.get(numTotalParticle - 1 - i);
			g.drawOval((int) p.getPose()[0] / 10 - 4, (int) p.getPose()[1] / 10 - 4, 8, 8);
		}
		
		/* Draw the robot & laser readings.
		 * Note that theta range is from -PI to PI. One laser reading spans 180 degrees (or PI); 
		 * hence, subtract PI/2 and draw lines by incrementing PI/180 radians.
		 */
		g.setColor(new Color(255, 0, 100));
		int x = (int) robot.getPose()[0];
		int y = (int) robot.getPose()[1];
		if (observation.dataType.equals("L")) {
			double laserTheta = robot.getPose()[2] - Math.PI / 2;
			for (double reading: observation.laserReadings) {
				int xt = x + (int) (reading * Math.cos(laserTheta));
				int yt = y + (int) (reading * Math.sin(laserTheta));
				g.drawLine(x / 10, y / 10, xt / 10, yt / 10);
				laserTheta += Math.PI / 180;
			}
		}
		g.setColor(new Color(100, 100, 255));
		g.fillOval((int) x / 10 - 5, (int) y / 10 - 5, 10, 10);
	}
}
