package dk.jonaslindstrom.boids;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.image.BufferStrategy;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import javax.swing.JFrame;
import javax.swing.Timer;

public class Boids extends Canvas {

  private final Flock flock;
  private final BufferStrategy strategy;
  private final Dimension dimension;

  public Boids(int w, int h, int n, int delay) {

    this.dimension = new Dimension(w, h);

    setPreferredSize(dimension);
    setBackground(Color.black);

    JFrame frame = new JFrame();
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.add(this);
    frame.pack();
    frame.setVisible(true);

    createBufferStrategy(2);
    strategy = getBufferStrategy();

    flock = new Flock(w, h, n);

    new Timer(delay, (ActionEvent e) -> super.repaint()).start();
  }

  public static void main(String[] args) {

    // Default configuration
    int w = 800;
    int h = 600;
    int n = 1000;
    int delay = 30;

    Properties prop = new Properties();
    String fileName = "boids.config";
    try {
      InputStream is = new FileInputStream(fileName);

      prop.load(is);

      w = Integer.parseInt(prop.getProperty("width"));
      h = Integer.parseInt(prop.getProperty("height"));
      n = Integer.parseInt(prop.getProperty("n"));
      delay = Integer.parseInt(prop.getProperty("delay"));

    } catch (FileNotFoundException ex) {
      System.out.println(
          "Configuration file, boids.config, was not found. Default configuration is used.");
    } catch (IOException e) {
      System.out.println("Error reading configuration file. Default configuration is used.");
      return;
    }

    new Boids(w, h, n, delay);
  }

  @Override
  public void update(Graphics g) {
    super.update(g);

    Graphics2D graphics2D = (Graphics2D) strategy.getDrawGraphics();
    graphics2D
        .setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    flock.run();

    graphics2D.setColor(Color.black);
    graphics2D.fillRect(0, 0, dimension.width, dimension.height);

    flock.draw(graphics2D);

    graphics2D.dispose();
    strategy.show();
  }

}

