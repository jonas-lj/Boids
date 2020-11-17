package dk.jonaslindstrom.boids.math;

import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;

public class Utils {

  /** If the magnitude of the given vector exceeds the limit, return a shortened version of the vector with length equal to limit. */
  public static <V> V limit(V vector, double limit, RealHilbertSpace<V> space) {
    double magnitude = space.norm(vector);
    if (magnitude > 0 && magnitude > limit) {
      return space.scale(limit / magnitude, vector);
    }
    return vector;
  }

  /** Return a vector with the same direction as <code>v</code> but with magnitude 1. */
  public static <V> V normalize(V v, RealHilbertSpace<V> space) {
    double magnitude = space.norm(v);
    if (magnitude == 0.0) {
      throw new IllegalArgumentException("The vector must be non-zero");
    }
    return space.scale(1.0 / magnitude, v);
  }

}
