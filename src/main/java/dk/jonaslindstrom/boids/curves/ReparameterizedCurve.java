package dk.jonaslindstrom.boids.curves;

import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class ReparameterizedCurve implements DoubleFunction<Vector2D> {

  private final DoubleFunction<Vector2D> curve;
  private final DoubleUnaryOperator reparameterization;

  public ReparameterizedCurve(DoubleFunction<Vector2D> curve,
      DoubleUnaryOperator reparameterization) {
    this.curve = curve;
    this.reparameterization = reparameterization;
  }

  @Override
  public Vector2D apply(double value) {
    return curve.apply(reparameterization.applyAsDouble(value));
  }
}
