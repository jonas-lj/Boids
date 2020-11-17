package dk.jonaslindstrom.boids.boid;

import dk.jonaslindstrom.boids.BoidClock;
import dk.jonaslindstrom.boids.BoidSettings;
import dk.jonaslindstrom.boids.math.Utils;
import dk.jonaslindstrom.boids.rules.Alignment;
import dk.jonaslindstrom.boids.rules.Cohesion;
import dk.jonaslindstrom.boids.rules.Predator;
import dk.jonaslindstrom.boids.rules.Separation;
import dk.jonaslindstrom.math.algebra.abstractions.RealHilbertSpace;
import org.apache.commons.math3.util.Pair;

/**
 * An implementation of Craig Raynolds' Boids algorithm dealing with alignment, cohesion, separation
 * and predators.
 *
 * @param <V>
 */
public class MyBoid<V> extends SimpleBoid<V, Pair<Double, Boolean>, BoidType> {

  private final BoidSettings<V> settings;
  private final RealHilbertSpace<V> space;

  public MyBoid(V location, V velocity, BoidSettings<V> settings,
      RealHilbertSpace<V> space, BoidClock clock) {
    this(location, velocity, false, settings, space, clock);
  }

  public MyBoid(V location, V velocity, boolean isPredator, BoidSettings<V> settings,
      RealHilbertSpace<V> space, BoidClock clock) {

    super(location, velocity, isPredator ? BoidType.PREDATOR : BoidType.BOID,
        (me, other) -> {
          double distance = space.distance(me.getLocation(), other.getLocation());
          boolean inView = inView(me, other, distance, space, settings.getSightDistance(),
              settings.getPeripheryAngle());
          return Pair.create(distance, inView);
        }, v -> {
          V withMigration = space.add(v, settings.getMigrate().apply(clock.getTime()));
          return Utils.limit(withMigration, settings.getMaxSpeed(), space);
        },
        space);

    this.settings = settings;
    this.space = space;
  }

  private static <W> boolean inView(Boid<W, ?> viewer,
      Boid<W, ?> target,
      double distance,
      RealHilbertSpace<W> space,
      double sightDistance,
      double peripheryAngle) {

    if (distance <= 0 || distance > sightDistance) {
      return false;
    }

    W lineOfSight = space.subtract(viewer.getLocation(), target.getLocation());

    double angle = space.angle(lineOfSight, viewer.getVelocity());
    return angle < peripheryAngle;
  }

  @Override
  public void init() {
    addRule(new Separation<>(
        settings.getDesiredSeparation(),
        settings.getMaxSpeed(),
        settings.getMaxForce(),
        space,
        b -> b.getType() == BoidType.BOID), settings.getSeparationCoefficient());

    addRule(new Alignment<>(
        settings.getFlockingDistance(),
        settings.getMaxSpeed(),
        settings.getMaxForce(),
        space,
        b -> b.getType() == BoidType.BOID), settings.getAlignmentCoefficient());

    addRule(new Cohesion<>(
            settings.getFlockingDistance(),
            settings.getMaxSpeed(),
            settings.getMaxForce(),
            space,
            b -> b.getType() == BoidType.BOID),
        settings.getCohesionCoefficient());

    addRule(new Predator<>(
            settings.getPredatorDistance(),
            settings.getMaxSpeed(),
            settings.getMaxForce(),
            space,
            b -> b.getType() == BoidType.PREDATOR),
        settings.getPredatorCoefficient());
  }
}
