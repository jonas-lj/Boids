package dk.jonaslindstrom.boids.rules;

import static dk.jonaslindstrom.boids.math.Utils.limit;
import static dk.jonaslindstrom.boids.math.Utils.normalize;

import dk.jonaslindstrom.boids.boid.Boid;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.util.function.Predicate;
import org.apache.commons.math3.util.Pair;

/**
 * Predator: Non-predator boids avoids nearby predators.
 */
public class Predator<V, T, B extends Boid<V, T>> implements Rule<V, Pair<Double, Boolean>, B> {

  private final double predatorDistance, maxSpeed, maxForce;
  private final RealHilbertSpace<V> space;
  private final Predicate<B> isPredator;
  private V target;
  private int count;

  public Predator(double predatorDistance, double maxSpeed, double maxForce,
      RealHilbertSpace<V> space, Predicate<B> isPredator) {
    this.predatorDistance = predatorDistance;
    this.maxSpeed = maxSpeed;
    this.maxForce = maxForce;
    this.space = space;
    this.isPredator = isPredator;
  }

  @Override
  public void reset() {
    this.target = space.getZero();
    this.count = 0;
  }

  @Override
  public boolean consider(B other, Pair<Double, Boolean> precomputed) {
    double distance = precomputed.getFirst();
    boolean inView = precomputed.getSecond();
    return isPredator.test(other) && inView && distance > 0 && distance < predatorDistance;
  }

  @Override
  public void process(B me, B other, Pair<Double, Boolean> precomputed) {
    if (isPredator.test(me)) {
      return;
    }
    target = space
        .add(target, normalize(space.subtract(me.getLocation(), other.getLocation()), space));
    count++;
  }

  @Override
  public V finish(B me) {
    return limit(target, maxForce, space);
  }

}
