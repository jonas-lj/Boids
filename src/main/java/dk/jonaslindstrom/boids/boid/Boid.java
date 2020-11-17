package dk.jonaslindstrom.boids.boid;

import java.util.stream.Stream;

/**
 * Instances of this interface represents a "Boid" (bird).
 *
 * @param <V> The vector type used by this boid.
 * @param <T> Class representing the boid type.
 */
public interface Boid<V, T> {

  /**
   * Code that has to be executed after creation but before iterations should be put here.
   */
  default void init() {
  }

  /**
   * Iterate the state of this boid. The given stream consists of all boids (including this) in the
   * flock.
   *
   * @param boids All boids in the flock.
   */
  void iterate(Stream<Boid<V, T>> boids);

  /**
   * Returns the location of this boid.
   */
  V getLocation();

  /**
   * Returns the current velocity of this boid.
   */
  V getVelocity();

  /**
   * Return the type of this boid.
   */
  T getType();

}
