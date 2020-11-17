package dk.jonaslindstrom.boids;

import static java.lang.Math.PI;

import dk.jonaslindstrom.boids.curves.LissajousCurve;
import dk.jonaslindstrom.boids.curves.ReparameterizedCurve;
import java.util.function.DoubleFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class BoidSettings<V> {

  private final double separationCoefficient;
  private final double alignmentCoefficient;
  private final double cohesionCoefficient;
  private final double maxSpeed;
  private final double maxForce;
  private final double sightDistance;
  private final double peripheryAngle;
  private final double desiredSeparation;
  private final double flockingDistance;
  private final double predatorDistance;
  private final DoubleFunction<V> migrate;
  private final double predatorCoefficient;

  public BoidSettings(
      double separationCoefficient,
      double alignmentCoefficient,
      double cohesionCoefficient,
      double predatorCoefficient,
      double maxSpeed,
      double maxForce,
      double sightDistance,
      double peripheryAngle,
      double desiredSeparation,
      double flockingDistance,
      double predatorDistance,
      DoubleFunction<V> migrate) {
    this.separationCoefficient = separationCoefficient;
    this.alignmentCoefficient = alignmentCoefficient;
    this.cohesionCoefficient = cohesionCoefficient;
    this.predatorCoefficient = predatorCoefficient;
    this.maxSpeed = maxSpeed;
    this.maxForce = maxForce;
    this.sightDistance = sightDistance;
    this.peripheryAngle = peripheryAngle;
    this.desiredSeparation = desiredSeparation;
    this.flockingDistance = flockingDistance;
    this.predatorDistance = predatorDistance;
    this.migrate = migrate;
  }

  public static BoidSettings<Vector2D> getDefault() {
    return new BoidSettings<>(
        2.8,
        1.5,
        1.1,
        6.2,
        3.5,
        0.07,
        150,
        PI,
        15,
        150,
        50,
        new ReparameterizedCurve(new LissajousCurve(3, 2, Math.PI / 2, 0.05), t -> 0.1 * t));
  }

  public double getSeparationCoefficient() {
    return separationCoefficient;
  }

  public double getAlignmentCoefficient() {
    return alignmentCoefficient;
  }

  public double getCohesionCoefficient() {
    return cohesionCoefficient;
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  public double getMaxForce() {
    return maxForce;
  }

  public double getSightDistance() {
    return sightDistance;
  }

  public double getPeripheryAngle() {
    return peripheryAngle;
  }

  public double getDesiredSeparation() {
    return desiredSeparation;
  }

  public double getFlockingDistance() {
    return flockingDistance;
  }

  public DoubleFunction<V> getMigrate() {
    return migrate;
  }

  public double getPredatorCoefficient() {
    return predatorCoefficient;
  }

  public double getPredatorDistance() {
    return predatorDistance;
  }
}
