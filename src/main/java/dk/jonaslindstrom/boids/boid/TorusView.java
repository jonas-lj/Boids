package dk.jonaslindstrom.boids.boid;

import java.awt.geom.Dimension2D;
import java.util.stream.Stream;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * This class wraps around a boid, replacing its location by an equivalent location on a torus
 * closest to a given boid.
 */
class TorusView implements Boid<Vector2D, BoidType> {

  private final Vector2D velocity;
  private final Vector2D location;
  private final BoidType type;

  TorusView(Boid<Vector2D, BoidType> origo, Boid<Vector2D, BoidType> boid, Dimension2D torus) {
    this.location = localCoordinate(origo.getLocation(), boid.getLocation(), torus.getWidth(),
        torus.getHeight());
    this.velocity = boid.getVelocity();
    this.type = boid.getType();
  }

  private static Vector2D localCoordinate(Vector2D origo, Vector2D target, double w, double h) {

    double x = target.getX();
    double y = target.getY();

    double dx = Math.abs(origo.getX() - x);
    double dy = Math.abs(origo.getY() - y);

    double dxCandidate = Math.abs(origo.getX() - (w + x));
    if (dxCandidate < dx) {
      x = w + x;
      dx = dxCandidate;
    }

    dxCandidate = Math.abs(origo.getX() - (x - w));
    if (dxCandidate < dx) {
      x = x - w;
    }

    double dyCandidate = Math.abs(origo.getY() - (h + y));
    if (dyCandidate < dy) {
      y = h + y;
      dy = dyCandidate;
    }

    dyCandidate = Math.abs(origo.getY() - (y - h));
    if (dyCandidate < dy) {
      y = y - h;
    }

    return new Vector2D(x, y);
  }

  @Override
  public void iterate(Stream<Boid<Vector2D, BoidType>> boids) {
    // Do nothing -- not to be used for processing other boids
  }

  @Override
  public Vector2D getLocation() {
    return location;
  }

  @Override
  public Vector2D getVelocity() {
    return velocity;
  }

  @Override
  public BoidType getType() {
    return type;
  }
}
