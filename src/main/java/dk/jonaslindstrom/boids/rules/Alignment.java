package dk.jonaslindstrom.boids.rules;

import static dk.jonaslindstrom.boids.math.Utils.limit;
import static dk.jonaslindstrom.boids.math.Utils.normalize;

import dk.jonaslindstrom.boids.boid.Boid;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.util.function.Predicate;
import org.apache.commons.math3.util.Pair;

/**
 * Alignment: A boid tries to move the same direction as other nearby boids.
 */
public class Alignment<V, T> implements Rule<V, Pair<Double, Boolean>, Boid<V, T>> {

  private final double flockingDistance, maxSpeed, maxForce;
  private final RealHilbertSpace<V> space;
  private final Predicate<Boid<V, T>> isBoid;
  private V steer;
  private int count;

  public Alignment(double flockingDistance, double maxSpeed, double maxForce,
      RealHilbertSpace<V> space, Predicate<Boid<V, T>> isBoid) {
    this.flockingDistance = flockingDistance;
    this.maxSpeed = maxSpeed;
    this.maxForce = maxForce;
    this.space = space;
    this.isBoid = isBoid;
  }

  @Override
  public void reset() {
    this.steer = space.getZero();
    this.count = 0;
  }

  @Override
  public boolean consider(Boid<V, T> other, Pair<Double, Boolean> precomputed) {
    double distance = precomputed.getFirst();
    boolean inView = precomputed.getSecond();
    return isBoid.test(other) && inView && distance > 0 && distance < flockingDistance;
  }

  @Override
  public void process(Boid<V, T> me, Boid<V, T> other, Pair<Double, Boolean> precomputed) {
    steer = space.add(steer, other.getVelocity());
    count++;
  }

  @Override
  public V finish(Boid<V, T> me) {
    if (count > 0) {
      steer = limit(space.subtract(
          space.scale(maxSpeed, normalize(space.scale(1.0 / count, steer), space)),
          me.getLocation()), maxForce, space);
    }
    return space.getZero();
  }

}
