package dk.jonaslindstrom.boids.rules;

import static dk.jonaslindstrom.boids.math.Utils.limit;
import static dk.jonaslindstrom.boids.math.Utils.normalize;

import dk.jonaslindstrom.boids.boid.Boid;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.util.function.Predicate;
import org.apache.commons.math3.util.Pair;

/**
 * Separation: Boids tries to avoid flying too close to other boids.
 */
public class Separation<V, T, B extends Boid<V, T>> implements Rule<V, Pair<Double, Boolean>, B> {

  private final double desiredSeparation, maxSpeed, maxForce;
  private final RealHilbertSpace<V> space;
  private final Predicate<B> isBoid;
  private V steer;
  private int count;

  public Separation(double desiredSeparation, double maxSpeed, double maxForce,
      RealHilbertSpace<V> space, Predicate<B> isBoid) {
    this.desiredSeparation = desiredSeparation;
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
  public boolean consider(B other, Pair<Double, Boolean> precomputed) {
    double distance = precomputed.getFirst();
    return isBoid.test(other) && distance > 0 && distance < desiredSeparation;
  }

  @Override
  public void process(B me, B other, Pair<Double, Boolean> precomputed) {
    double distance = precomputed.getFirst();
    V diff = space.scale(1.0 / distance,
        normalize(space.subtract(me.getLocation(), other.getLocation()), space));
    steer = space.add(steer, diff);
    count++;
  }

  @Override
  public V finish(B me) {

    if (count > 0) {
      steer = space.scale(1.0 / count, steer);
    }

    if (space.norm(steer) > 0) {
      return limit(
          space.subtract(space.scale(maxSpeed, normalize(steer, space)), me.getVelocity()),
          maxForce, space);
    }
    return space.getZero();
  }

}
