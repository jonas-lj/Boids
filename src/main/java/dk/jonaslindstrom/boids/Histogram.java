package dk.jonaslindstrom.boids;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.function.Predicate;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.Pair;

public class Histogram<T> {

  private final List<Predicate<T>> predicates;
  private final AtomicIntegerArray counts;

  public Histogram(List<Predicate<T>> predicates) {
    this.predicates = predicates;
    this.counts = new AtomicIntegerArray(predicates.size());
  }

  public static Histogram<Vector2D> createTwoDimensionalHistogram(double[] horizontal,
      double[] vertical) {
    List<Predicate<Vector2D>> predicates = new ArrayList<>();
    for (int j = 1; j < vertical.length; j++) {
      for (int i = 1; i < horizontal.length; i++) {
        int finalI = i;
        int finalJ = j;
        predicates.add(v -> v.getX() >= horizontal[finalI - 1] && v.getX() < horizontal[finalI]
            && v.getY() >= vertical[finalJ - 1] && v.getY() < vertical[finalJ]);
      }
    }
    return new Histogram<>(predicates);
  }

  public void handle(T value) {
    for (int i = 0; i < predicates.size(); i++) {
      if (predicates.get(i).test(value)) {
        counts.incrementAndGet(i);
      }
    }
  }

  public int getCount(int index) {
    return counts.get(index);
  }

  public void reset() {
    for (int i = 0; i < counts.length(); i++) {
      counts.set(i, 0);
    }
  }

  public int[] getRanking() {
    List<Pair<Integer, Integer>> ranking = new ArrayList<>();
    for (int i = 0; i < counts.length(); i++) {
      ranking.add(Pair.create(i, counts.get(i)));
    }
    ranking.sort((a, b) -> -Integer.compare(a.getSecond(), b.getSecond()));
    return ranking.stream().mapToInt(Pair::getFirst).toArray();
  }

}
