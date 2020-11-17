package dk.jonaslindstrom.boids.boid;

import dk.jonaslindstrom.boids.rules.Rule;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.UnaryOperator;
import java.util.stream.Stream;
import org.apache.commons.math3.util.Pair;

/**
 * An abstract implementation of a boid which handles other boids according to certain rules. All
 * implementations of boids should propably extend this class.
 *
 * @param <V> Vector type.
 * @param <S> Class containing precomputations to be reused by all rules.
 * @param <T> Boid type.
 */
public class SimpleBoid<V, S, T> implements Boid<V, T> {

  private final List<Pair<Rule<V, S, Boid<V, T>>, Double>> rules = new ArrayList<>();
  private final RealHilbertSpace<V> space;
  private final BiFunction<Boid<V, T>, Boid<V, T>, S> precomputation;
  private final UnaryOperator<V> postProcessing;
  private final T type;
  protected V location;
  private V velocity;

  public SimpleBoid(V location,
      V velocity,
      T type,
      BiFunction<Boid<V, T>, Boid<V, T>, S> precomputation,
      UnaryOperator<V> postProcessing,
      RealHilbertSpace<V> space) {
    this.space = space;
    this.location = location;
    this.type = type;
    this.precomputation = precomputation;
    this.postProcessing = postProcessing;
    this.velocity = velocity;
  }

  public void addRule(Rule<V, S, Boid<V, T>> rule, double coefficient) {
    rules.add(Pair.create(rule, coefficient));
  }

  @Override
  public void iterate(Stream<Boid<V, T>> boids) {
    for (Pair<Rule<V, S, Boid<V, T>>, Double> rule : rules) {
      rule.getFirst().reset();
    }

    boids.sequential().forEach(boid -> {
      S precomputed = precomputation.apply(this, boid);
      for (Pair<Rule<V, S, Boid<V, T>>, Double> rule : rules) {
        if (rule.getFirst().consider(boid, precomputed)) {
          rule.getFirst().process(this, boid, precomputed);
        }
      }
    });

    V acceleration = space.getZero();
    for (Pair<Rule<V, S, Boid<V, T>>, Double> rule : rules) {
      V contribution = space.scale(rule.getSecond(), rule.getFirst().finish(this));
      acceleration = space.add(acceleration, contribution);
    }

    velocity = postProcessing.apply(space.add(velocity, acceleration));
    location = space.add(location, velocity);
  }

  @Override
  public V getLocation() {
    return location;
  }

  @Override
  public V getVelocity() {
    return velocity;
  }

  @Override
  public T getType() {
    return type;
  }

}
