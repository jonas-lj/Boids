package dk.jonaslindstrom.boids.rules;

import static dk.jonaslindstrom.boids.math.Utils.limit;
import static dk.jonaslindstrom.boids.math.Utils.normalize;

import dk.jonaslindstrom.boids.boid.Boid;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.util.function.Predicate;
import org.apache.commons.math3.util.Pair;

/**
 * Cohesion: A boid tries to near the center of nearby boids.
 */
public class Cohesion<V, T, B extends Boid<V, T>> implements Rule<V, Pair<Double, Boolean>, B> {

  private final double flockingDistance, maxSpeed, maxForce;
  private final RealHilbertSpace<V> space;
  private final Predicate<B> isBoid;
  private V target;
  private int count;

  public Cohesion(double flockingDistance, double maxSpeed, double maxForce,
      RealHilbertSpace<V> space, Predicate<B> isBoid) {
    this.flockingDistance = flockingDistance;
    this.maxSpeed = maxSpeed;
    this.maxForce = maxForce;
    this.space = space;
    this.isBoid = isBoid;
  }

  @Override
  public void reset() {
    this.target = space.getZero();
    this.count = 0;
  }

  @Override
  public boolean consider(B other, Pair<Double, Boolean> precomputed) {
    boolean inView = precomputed.getSecond();
    double distance = precomputed.getFirst();
    return isBoid.test(other) && inView && distance > 0 && distance < flockingDistance;
  }

  @Override
  public void process(B me, B other, Pair<Double, Boolean> precomputed) {
    target = space.add(target, other.getLocation());
    count++;
  }

  @Override
  public V finish(B me) {
    if (count > 0) {
      target = space.scale(1.0 / count, target);
      return seek(me.getLocation(), target, me.getVelocity());
    }
    return space.getZero();

  }

  private V seek(V from, V target, V currentVelocity) {
    return limit(
        space.subtract(space.scale(maxSpeed,
            normalize(space.subtract(target, from), space)), currentVelocity), maxForce,
        space);
  }

}
