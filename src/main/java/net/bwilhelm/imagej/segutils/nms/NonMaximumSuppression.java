/*-
 * #%L
 * Classes for working with ROIs. Will be contributed to the appropriate imagej and imglib projects.
 * %%
 * Copyright (C) 2019 Benjamin Wilhelm
 * %%
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * #L%
 */
package net.bwilhelm.imagej.segutils.nms;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ConcurrentLinkedQueue;

import de.lighti.clipper.Clipper;
import de.lighti.clipper.Clipper.ClipType;
import de.lighti.clipper.Clipper.PolyFillType;
import de.lighti.clipper.Clipper.PolyType;
import de.lighti.clipper.DefaultClipper;
import de.lighti.clipper.Path;
import de.lighti.clipper.Paths;
import de.lighti.clipper.Point.LongPoint;
import net.bwilhelm.imagej.segutils.rois.Box2D;
import net.imglib2.RealInterval;
import net.imglib2.RealLocalizable;
import net.imglib2.roi.geom.real.Box;
import net.imglib2.roi.geom.real.Polygon2D;
import net.imglib2.util.Pair;

/**
 * @author Benjamin Wilhelm
 */
public class NonMaximumSuppression {

	private static final long DOUBLE_TO_LONG_FACTOR = 1000000;

	private NonMaximumSuppression() {
		// Utility class
	}

	/**
	 * Selects a list of ROIs in descending order of score. ROIs that have an
	 * intersection-over-union larger than the <code>iouThreshold</code> with a
	 * previously selected ROI are not selected.
	 *
	 * @param rois         the ROIs as a pair of polygons and scores
	 * @param iouThreshold the threshold for the intersection-over-union
	 * @return the maximum ROIs
	 */
	public static List<Pair<Polygon2D, Double>> nonMaximumSuppressionIoU(final List<Pair<Polygon2D, Double>> rois,
			final double iouThreshold) {
		return nonMaximumSuppression(rois, (i, a, b) -> (i / (a + b - i)) >= iouThreshold);
	}

	/**
	 * Selects a list of ROIs in descending order of score. ROIs that have an
	 * overlap larger than the <code>overlapThreshold</code> with a previously
	 * selected ROI are not selected. The overlap is computed by dividing the the
	 * area of the intersection by the area of the smaller ROI.
	 *
	 * @param rois             the ROIs as a pair of polygons and scores
	 * @param overlapThreshold the overlap threshold
	 * @return the maximum ROIs
	 */
	public static List<Pair<Polygon2D, Double>> nonMaximumSuppressionOverlap(final List<Pair<Polygon2D, Double>> rois,
			final double overlapThreshold) {
		return nonMaximumSuppression(rois, (i, a, b) -> {
			if (a < b) {
				return (i / (a + 1.e-10)) > overlapThreshold;
			} else {
				return (i / (b + 1.e-10)) > overlapThreshold;
			}
		});
	}

	/**
	 * Selects a list of ROIs in descending order of score. ROIs for which the
	 * <code>suppressionCriterion</code> with a previously selected ROI is true are
	 * not selected.
	 *
	 * @param rois                 the ROIs as a pair of polygons and scores
	 * @param suppressionCriterion a function which decides if the ROI should be
	 *                             suppressed
	 * @return the maximum ROIs
	 */
	public static List<Pair<Polygon2D, Double>> nonMaximumSuppression(final List<Pair<Polygon2D, Double>> rois,
			final SuppressionCriterion suppressionCriterion) {

		// Sort the rois by score
		Collections.sort(rois, NonMaximumSuppression::compareScores);

		// Copy the rois into a linked list
		final Queue<Pair<Polygon2D, Double>> queue = new ConcurrentLinkedDeque<>(rois);

		final List<Pair<Polygon2D, Double>> r = new LinkedList<>();

		while (!queue.isEmpty()) {
			final Pair<Polygon2D, Double> valA = queue.poll();
			r.add(valA);

			// Get the polygon and area of the current roi
			final Polygon2D roiA = valA.getA();
			final double areaA = area(roiA);

			// Compare against each polygon left with a lower score
			final Queue<Pair<Polygon2D, Double>> restQueue = new ConcurrentLinkedQueue<>(queue);
			while (!restQueue.isEmpty()) {
				final Pair<Polygon2D, Double> valB = restQueue.poll();
				final Polygon2D roiB = valB.getA();

				// Compute the intersection
				final double intersectionArea = intersectionArea(roiA, roiB);

				// Check the suppression criterion (only if the intersection area is larger than
				// 0)
				if (intersectionArea > 0) {
					final double areaB = area(roiB);
					if (suppressionCriterion.suppress(intersectionArea, areaA, areaB)) {
						queue.remove(valB);
					}
				}
			}
		}
		return r;
	}

	/** Compares the scores of two ROIs */
	private static int compareScores(final Pair<?, Double> l, final Pair<?, Double> r) {
		if (l.getB() > r.getB()) {
			return -1;
		} else if (l.getB() < r.getB()) {
			return 1;
		} else {
			return 0;
		}
	}

	/**
	 * Calculates the area of the intersection of the two Polygons. Uses more
	 * efficient functions if the polygons are of types that can be intersected
	 * easily.
	 */
	private static double intersectionArea(final Polygon2D a, final Polygon2D b) {
		// Check the bounding box first
		if (!(intersects(a, b, 0) && intersects(a, b, 1))) {
			return 0;
		}

		// TODO ImageJ-Ops should be used to compute the intersection area.
		if (a instanceof Box2D && b instanceof Box2D) {
			return intersectionArea((Box2D) a, (Box2D) b);
		} else {
			return intersectionAreaClipper(a, b);
		}
	}

	private static boolean intersects(final RealInterval a, final RealInterval b, final int d) {
		return a.realMin(d) < b.realMax(d) && b.realMin(d) < a.realMax(d);
	}

	/** Computes the area of the given polygon */
	private static double area(final Polygon2D p) {
		if (p instanceof Box2D) {
			return ((Box) p).sideLength(0) * ((Box) p).sideLength(1);
		} else {
			if (p.numVertices() < 3) {
				return 0;
			}
			// TODO this is only true for non self-intersecting polygons
			double area = 0;
			final int numVertices = p.numVertices();
			for (int i = 0; i < numVertices; i++) {
				final RealLocalizable v1 = p.vertex(i);
				final RealLocalizable v2 = p.vertex((i + 1) % numVertices);
				// v1.X * v2.Y - v1.Y * v2.X
				area += v1.getDoublePosition(0) * v2.getDoublePosition(1)
						- v1.getDoublePosition(1) * v2.getDoublePosition(0);
			}
			return 0.5 * Math.abs(area);
		}
	}

	/**
	 * Computes the area of intersection for rectangles. Expects the rectangles to
	 * intersect
	 */
	private static double intersectionArea(final Box2D a, final Box2D b) {
		final RealLocalizable aMin = a.vertex(0);
		final RealLocalizable aMax = a.vertex(2);
		final RealLocalizable bMin = b.vertex(0);
		final RealLocalizable bMax = b.vertex(2);
		// Compute the width of the intersection
		final double width = intersectionLength(aMin.getDoublePosition(0), bMin.getDoublePosition(0),
				aMax.getDoublePosition(0), bMax.getDoublePosition(0));
		// Compute the height of the intersection
		final double height = intersectionLength(aMin.getDoublePosition(1), bMin.getDoublePosition(1),
				aMax.getDoublePosition(1), bMax.getDoublePosition(1));
		return width * height;
	}

	private static double intersectionAreaClipper(final Polygon2D a, final Polygon2D b) {
		// TODO implement this without clipper
		final Clipper clipper = new DefaultClipper();
		final Path aPath = convertToPath(a);
		final Path bPath = convertToPath(b);
		clipper.addPath(aPath, PolyType.CLIP, true);
		clipper.addPath(bPath, PolyType.SUBJECT, true);
		final Paths solution = new Paths();
		clipper.execute(ClipType.INTERSECTION, solution, PolyFillType.NON_ZERO, PolyFillType.NON_ZERO);
		double totalArea = 0;
		for (final Path p : solution) {
			// Logic from area function: But don't want to create other objects
			double area = 0;
			final int numVertices = p.size();
			for (int i = 0; i < numVertices; i++) {
				final int next = (i + 1) % numVertices;
				area += (double) (p.get(i).getX() * p.get(next).getY() - p.get(i).getY() * p.get(next).getX())
						/ (DOUBLE_TO_LONG_FACTOR * DOUBLE_TO_LONG_FACTOR);
			}
			totalArea += 0.5 * Math.abs(area);
		}
		return totalArea;
	}

	private static Path convertToPath(final Polygon2D p) {
		final int numVertices = p.numVertices();
		final Path path = new Path(numVertices);
		for (int i = 0; i < numVertices; i++) {
			final RealLocalizable v = p.vertex(i);
			path.add(new LongPoint((long) (v.getDoublePosition(0) * DOUBLE_TO_LONG_FACTOR),
					(long) (v.getDoublePosition(1) * DOUBLE_TO_LONG_FACTOR)));
		}
		return path;
	}

	private static double intersectionLength(final double aMin, final double bMin, final double aMax,
			final double bMax) {
		// Don't intersect
		if (aMin >= bMax || bMin >= aMax) {
			return 0;
		}
		final double min = aMin > bMin ? aMin : bMin;
		final double max = aMax < bMax ? aMax : bMax;
		return max - min;
	}

	/**
	 * Represents a function that decides if a ROI should be suppressed or not. See
	 * {@link #suppress(double, double, double)}.
	 */
	@FunctionalInterface
	public interface SuppressionCriterion {

		/**
		 * Decides if a ROI should be suppressed.
		 *
		 * @param intersection the area of intersection between the ROI and the
		 *                     previously selected ROI
		 * @param areaA        the area of the previously selected ROI
		 * @param areaB        the area of the ROI
		 * @return true if the ROI should be suppressed.
		 */
		boolean suppress(double intersection, double areaA, double areaB);
	}
}
