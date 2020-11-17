package dk.jonaslindstrom.boids.boid;

import dk.jonaslindstrom.boids.BoidClock;
import dk.jonaslindstrom.boids.BoidSettings;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.awt.geom.Dimension2D;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.Stream;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;

/**
 * This class represents a boid on a torus, e.g. a plane where the sides are glued together. It
 * works exactly as {@link MyBoid} but handles wrap-around issues.
 */
public class TorusBoid extends MyBoid<Vector2D> {

  private final Dimension2D torus;
  private final double linf;

  public TorusBoid(Vector2D location,
      Vector2D velocity,
      boolean isPredator,
      BoidSettings<Vector2D> settings,
      RealHilbertSpace<Vector2D> space,
      Dimension2D torus,
      BoidClock clock) {
    super(location, velocity, isPredator, settings, space, clock);
    this.torus = torus;
    this.linf = settings.getFlockingDistance();
  }

  public TorusBoid(Vector2D location,
      Vector2D velocity,
      BoidSettings<Vector2D> settings,
      RealHilbertSpace<Vector2D> space,
      Dimension2D torus,
      BoidClock clock) {
    this(location, velocity, false, settings, space, torus, clock);
  }

  /**
   * Apply the given operation to the value until the goal is true.
   */
  private static double ensure(double value, DoublePredicate goal, DoubleUnaryOperator operation) {
    while (!goal.test(value)) {
      value = operation.applyAsDouble(value);
    }
    return value;
  }

  @Override
  public void iterate(Stream<Boid<Vector2D, BoidType>> boids) {
    Stream<Boid<Vector2D, BoidType>> mapped = boids.map(b -> new TorusView(this, b, torus));
    mapped = mapped.filter(b -> FastMath
        .abs(b.getLocation().getX() - location.getX()) < linf && FastMath
        .abs(b.getLocation().getY() - location.getY()) < linf);
    super.iterate(mapped);

    this.location = ensureOnTorus(this.location);
  }

  private Vector2D ensureOnTorus(Vector2D location) {
    if (location.getX() < 0 || location.getX() > torus.getWidth() ||
        location.getY() < 0 || location.getY() > torus.getHeight()) {
      double x = location.getX();
      x = ensure(x, v -> v >= 0, v -> v + torus.getWidth());
      x = ensure(x, v -> v < torus.getWidth(), v -> v - torus.getWidth());

      double y = location.getY();
      y = ensure(y, v -> v >= 0, v -> v + torus.getHeight());
      y = ensure(y, v -> v < torus.getWidth(), v -> v - torus.getHeight());
      return new Vector2D(x, y);
    }
    return location;
  }

}
