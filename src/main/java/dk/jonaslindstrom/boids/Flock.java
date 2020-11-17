package dk.jonaslindstrom.boids;

import dk.jonaslindstrom.boids.boid.Boid;
import dk.jonaslindstrom.boids.boid.BoidType;
import dk.jonaslindstrom.boids.boid.CurveBoid;
import dk.jonaslindstrom.boids.boid.TorusBoid;
import dk.jonaslindstrom.boids.curves.Circle;
import dk.jonaslindstrom.boids.curves.ReparameterizedCurve;
import dk.jonaslindstrom.boids.math.DefaultSpace;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Dimension2D;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;

class Flock {

  private final static Random random = new Random(1234);
  private final static RealHilbertSpace<Vector2D> space = new DefaultSpace();
  private static final Shape shape = new Ellipse2D.Double(-2, -2, 4, 4);
  private final List<Boid<Vector2D, BoidType>> boids;
  private final BoidClock clock;
  private final double w;
  private final double h;
  private final int numBoids;

  Flock(double w, double h, int numBoids) {
    boids = new ArrayList<>();
    clock = new BoidClock(0.01);
    this.w = w;
    this.h = h;
    this.numBoids = numBoids;
    respawn();
  }

  public Stream<Vector2D> getBoids() {
    return boids.stream().map(Boid::getLocation);
  }

  private void respawn() {
    boids.clear();

    Dimension2D torus = new Dimension((int) w, (int) h);

    boids.add(new CurveBoid<>(new ReparameterizedCurve(new Circle(new Vector2D(350, 350), 150),
        t -> 2.0 * t),
        clock,
        BoidType.PREDATOR));
    for (int i = 1; i < numBoids; i++) {
      Boid<Vector2D, BoidType> boid = new TorusBoid(new Vector2D(w * random.nextDouble(),
          h * random.nextDouble()),
          new Vector2D(Math.acos(random.nextDouble()),
              Math.asin(random.nextDouble())),
          BoidSettings.getDefault(),
          space,
          torus,
          clock);
      boid.init();
      boids.add(boid);
    }
  }

  public void run() {
    clock.iterate();
    boids.stream().parallel().forEach(b -> {
      b.iterate(boids.stream());
    });
  }

  public void draw(Graphics2D g) {
    boids.stream().sequential().forEach(b -> {
      drawBoid(b, g);
    });
  }

  synchronized private void drawBoid(Boid<Vector2D, BoidType> b, Graphics2D g) {
    AffineTransform save = g.getTransform();
    g.translate(b.getLocation().getX(), b.getLocation().getY());
    g.rotate(heading(b));
    g.setColor(b.getType() == BoidType.PREDATOR ? Color.red : Color.white);
    g.fill(shape);
    g.drawLine(0,0, 5, 0);
    g.setTransform(save);
  }

  private double heading(Boid<Vector2D, BoidType> b) {
    return FastMath.atan2(b.getVelocity().getY(), b.getVelocity().getX());
  }

}
