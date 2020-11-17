package dk.jonaslindstrom.boids.curves;

import java.util.function.DoubleFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;

public class Circle implements DoubleFunction<Vector2D> {

  private final Vector2D center;
  private final double radius;

  public Circle(Vector2D center, double radius) {
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Vector2D apply(double t) {
    return new Vector2D(radius * FastMath.cos(t) + center.getX(),
        radius * FastMath.sin(t) + center.getY());
  }
}
