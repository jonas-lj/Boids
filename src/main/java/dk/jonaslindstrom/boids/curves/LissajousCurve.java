package dk.jonaslindstrom.boids.curves;

import java.util.function.DoubleFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;

public class LissajousCurve implements DoubleFunction<Vector2D> {

  private final double a, b, δ, s;

  public LissajousCurve(double a, double b, double δ, double s) {
    this.a = a;
    this.b = b;
    this.δ = δ;
    this.s = s;
  }

  @Override
  public Vector2D apply(double t) {
    return new Vector2D(s * FastMath.sin(2.0 * FastMath.PI * a * t + δ),
        s * FastMath.cos(2.0 * FastMath.PI * b * t));
  }

}
