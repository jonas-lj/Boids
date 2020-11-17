package dk.jonaslindstrom.boids.rules;

import dk.jonaslindstrom.boids.boid.Boid;

/**
 * A rule on how to a boid should act depending on the other boids on the flock.
 */
public interface Rule<V, S, B extends Boid<V, ?>> {

  /**
   * This rule is re-used for all boids in the flock and is reset before each iteration using this
   * method.
   */
  void reset();

  /**
   * Should the given boid be considered?
   */
  boolean consider(B other, S precomputed);

  /**
   * Process boid.
   */
  void process(B me, B other, S precomputed);

  /**
   * After the iteration, this method is called and should return this rules contribution to the
   * velocity of the boid.
   */
  V finish(B me);
}
