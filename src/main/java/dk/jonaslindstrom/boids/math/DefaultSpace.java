package dk.jonaslindstrom.boids.math;

import dk.jonaslindstrom.math.algebra.abstractions.AdditiveGroup;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class DefaultSpace extends RealHilbertSpace<Vector2D> {

  public DefaultSpace() {
    super(new AdditiveGroup<>() {

      @Override
      public String toString(Vector2D v) {
        return v.toString();
      }

      @Override
      public boolean equals(Vector2D v, Vector2D u) {
        return v.equals(u);
      }

      @Override
      public Vector2D add(Vector2D v, Vector2D u) {
        return v.add(u);
      }

      @Override
      public Vector2D negate(Vector2D v) {
        return v.negate();
      }

      @Override
      public Vector2D getZero() {
        return Vector2D.ZERO;
      }
    });
  }

  @Override
  public Double innerProduct(Vector2D v, Vector2D u) {
    return v.dotProduct(u);
  }

  @Override
  public Vector2D scale(Double s, Vector2D v) {
    return v.scalarMultiply(s);
  }

}
