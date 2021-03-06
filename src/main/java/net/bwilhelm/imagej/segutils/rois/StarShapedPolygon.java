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

import net.imglib2.RealLocalizable;
import net.imglib2.roi.geom.real.Polygon2D;

/**
 * Star-shaped polygon. Defined by a center position and an angel and length for
 * each vertex.
 *
 * @author Benjamin Wilhelm
 */
public interface StarShapedPolygon extends Polygon2D {

	/**
	 * @return the center of this polygon. This is a point from which the entire
	 *         polygon boundary is visible.
	 */
	RealLocalizable center();

	/**
	 * @param pos the position
	 * @return the length of the ray in the specified position
	 */
	double rayLength(int pos);

	/**
	 * @param pos the position
	 * @return the angle of the ray in the specified position. A point that is
	 *         exactly east of the center has the angle 0.
	 */
	double rayAngle(int pos);
}
