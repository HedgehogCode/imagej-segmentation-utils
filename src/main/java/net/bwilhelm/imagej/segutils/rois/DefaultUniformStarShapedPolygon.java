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
package net.bwilhelm.imagej.segutils.rois;

import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import net.imglib2.AbstractRealInterval;
import net.imglib2.RealLocalizable;
import net.imglib2.RealPoint;
import net.imglib2.roi.BoundaryType;

/**
 * Default implementation of a star-shaped polygon that has an equal angel
 * between each point. Therefore it can be defined only by a center coordinate
 * and a distance for each vertex.
 *
 * @author Benjamin Wilhelm
 */
public class DefaultUniformStarShapedPolygon extends AbstractRealInterval implements StarShapedPolygon {

	/** The center of the star shaped polygon */
	protected final RealLocalizable m_center;

	/** The number of vertices */
	protected final int m_numVertices;

	/** The distances from the center to the vertices */
	protected final TDoubleArrayList m_distances;

	/** If the boundary is included in the polygon */
	protected final boolean m_closed;

	/**
	 * Creates a new {@link StarShapedPolygon} where the angle between each point is
	 * fixed to 2 * PI / number of vertices. The first given vertex is east of the
	 * center.
	 *
	 * @param center    the center of the polygon
	 * @param distances the distance from the center to each point
	 * @param closed    if the boundary should be included in the polygon
	 */
	public DefaultUniformStarShapedPolygon(final RealLocalizable center, final List<Double> distances,
			final boolean closed) {
		super(2);
		this.m_center = center;
		this.m_numVertices = distances.size();
		this.m_distances = new TDoubleArrayList(distances.size());
		this.m_closed = closed;
		populateDistances(distances);
	}

	/**
	 * Creates a new closed {@link StarShapedPolygon} where the angle between each
	 * point is fixed to 2 * PI / number of vertices. The first given vertex is east
	 * of the center.
	 *
	 * @param center    the center of the polygon
	 * @param distances the distance from the center to each point
	 */
	public DefaultUniformStarShapedPolygon(final RealLocalizable center, final List<Double> distances) {
		this(center, distances, true);
	}

	@Override
	public RealLocalizable vertex(final int pos) {
		return computePosition(pos, m_distances.get(pos));
	}

	@Override
	public int numVertices() {
		return m_numVertices;
	}

	@Override
	public boolean test(final RealLocalizable point) {
		// Calculate the angel from the center
		final double relX = point.getDoublePosition(0) - m_center.getDoublePosition(0);
		final double relY = point.getDoublePosition(1) - m_center.getDoublePosition(1);
		double angle = Math.atan2(relY, relX);
		angle = angle > 0 ? angle : Math.PI * 2 + angle;
		final double pointDistance = Math.sqrt(relX * relX + relY * relY);

		// Find the indices of the according polygon points
		final double index = angle * m_numVertices / (2 * Math.PI);
		final int idx1 = (int) Math.floor(index) % m_numVertices;
		final int idx2 = (int) Math.ceil(index) % m_numVertices;

		// First check: Point lies on the same line as a vertex:
		if (idx1 == idx2) {
			if (m_closed) {
				return pointDistance <= m_distances.get(idx1);
			} else {
				return pointDistance < m_distances.get(idx1);
			}
		}

		// Second check: Compute the distance of the border at this angel and compare
		final double borderDistance = m_distances.get(idx1)
				+ (index - idx1) * (m_distances.get(idx2) - m_distances.get(idx1));
		if (m_closed) {
			return pointDistance <= borderDistance;
		} else {
			return pointDistance < borderDistance;
		}
	}

	@Override
	public RealLocalizable center() {
		return m_center;
	}

	@Override
	public double rayLength(final int pos) {
		return m_distances.get(pos);
	}

	@Override
	public double rayAngle(final int pos) {
		return pos * (2 * Math.PI / m_numVertices);
	}

	@Override
	public BoundaryType boundaryType() {
		if (m_closed) {
			return BoundaryType.CLOSED;
		} else {
			return BoundaryType.OPEN;
		}
	}

	/**
	 * Populates the distances array, and sets min/max values.
	 *
	 * @param dists contains the distances for each vertex
	 */
	private void populateDistances(final List<Double> dists) {
		double minX = Double.POSITIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY;
		double maxX = Double.NEGATIVE_INFINITY;
		double maxY = Double.NEGATIVE_INFINITY;

		for (int i = 0; i < dists.size(); i++) {
			final double d = dists.get(i);
			m_distances.add(d);
			final RealLocalizable pos = computePosition(i, d);
			final double xi = pos.getDoublePosition(0);
			final double yi = pos.getDoublePosition(1);
			if (xi > maxX) {
				maxX = xi;
			}
			if (xi < minX) {
				minX = xi;
			}
			if (yi > maxY) {
				maxY = yi;
			}
			if (yi < minY) {
				minY = yi;
			}
		}
		max[0] = maxX;
		max[1] = maxY;
		min[0] = minX;
		min[1] = minY;
	}

	/**
	 * Computes the position of a vertex given its index (determining the angle) and
	 * its distance from the center.
	 */
	private RealPoint computePosition(final int index, final double distance) {
		final double angle = rayAngle(index);
		final double x = m_center.getDoublePosition(0) + Math.cos(angle) * distance;
		final double y = m_center.getDoublePosition(1) + Math.sin(angle) * distance;
		return new RealPoint(x, y);
	}
}
