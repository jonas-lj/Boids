package dk.jonaslindstrom.boids;

/**
 * This class represents a clock used by some boids to change their behaviour over time.
 */
public class BoidClock {

  private final double delta;
  private double t;

  public BoidClock(double delta) {
    this.delta = delta;
    this.t = 0;
  }

  void iterate() {
    t += delta;
  }

  public double getTime() {
    return t;
  }

}
