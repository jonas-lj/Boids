package dk.jonaslindstrom.boids.boid;

import dk.jonaslindstrom.boids.BoidClock;
import java.util.function.DoubleFunction;
import java.util.stream.Stream;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * A Boid following a fixed curve, ignoring all other boids.
 *
 * @param <T> Boid type.
 */
public class CurveBoid<T> implements Boid<Vector2D, T> {

  private final DoubleFunction<Vector2D> curve;
  private final BoidClock clock;
  private final T type;
  private Vector2D location, velocity;

  public CurveBoid(DoubleFunction<Vector2D> curve, BoidClock clock, T type) {
    this.curve = curve;
    this.clock = clock;
    this.location = curve.apply(0.0);
    this.velocity = Vector2D.ZERO;
    this.type = type;
  }

  @Override
  public void iterate(Stream<Boid<Vector2D, T>> boids) {
    Vector2D newLocation = curve.apply(clock.getTime());
    this.velocity = newLocation.subtract(location);
    this.location = newLocation;
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
  public T getType() {
    return type;
  }


}
