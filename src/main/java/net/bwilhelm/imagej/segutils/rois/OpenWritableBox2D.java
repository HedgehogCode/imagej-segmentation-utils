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
import net.imglib2.roi.BoundaryType;

/**
 * @author Benjamin Wilhelm
 */
public class OpenWritableBox2D extends AbstractWritableBox2D {

	/**
	 * Creates a new two-dimensional writable box (e.g. a rectangle) with the given
	 * corner positions.
	 *
	 * @param minX min in first dimension
	 * @param minY min in second dimension
	 * @param maxX max in first dimension
	 * @param maxY max in second dimension
	 */
	public OpenWritableBox2D(final double minX, final double minY, final double maxX, final double maxY) {
		super(minX, minY, maxX, maxY);
	}

	@Override
	public boolean test(final RealLocalizable t) {
		final double x = t.getDoublePosition(0);
		final double y = t.getDoublePosition(1);
		return x > min[0] && x < max[0] && y > min[1] && y < max[1];
	}

	@Override
	public BoundaryType boundaryType() {
		return BoundaryType.OPEN;
	}
}
