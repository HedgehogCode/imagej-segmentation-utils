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
import net.imglib2.RealPoint;
import net.imglib2.roi.geom.real.AbstractWritableBox;

/**
 * @author Benjamin Wilhelm
 */
public abstract class AbstractWritableBox2D extends AbstractWritableBox implements WritableBox2D {

	/**
	 * Creates a new two-dimensional writable box (e.g. a rectangle) with the given
	 * corner positions.
	 *
	 * @param minX min in first dimension
	 * @param minY min in second dimension
	 * @param maxX max in first dimension
	 * @param maxY max in second dimension
	 */
	public AbstractWritableBox2D(final double minX, final double minY, final double maxX, final double maxY) {
		super(new double[] { minX, minY }, new double[] { maxX, maxY });
	}

	@Override
	public RealLocalizable vertex(final int pos) {
		/*
		 * 0----1 | | 3----2
		 */
		final double x = pos == 0 || pos == 3 ? min[0] : max[0];
		final double y = pos == 0 || pos == 1 ? min[1] : max[1];
		return new RealPoint(x, y);
	}

	@Override
	public int numVertices() {
		return 4;
	}
}
