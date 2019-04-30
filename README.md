NOTE: This repository is an intermediate point for code that should be contributed to imglib/imglib2-roi and imagej/imagej-ops.

# Utilities for segmentation in the imagej/imglib2 ecosystem

## Quickstart

Clone and install Clipper in you local maven repository:
```sh
git clone https://github.com/lightbringer/clipper-java.git
cd clipper-java/Clipper
mvn install
```

Clone build the segmentation utils:
```sh
git clone https://github.com/HedgehogCode/imagej-segmentation-utils
cd imagej-segmentation-utils
mvn
```

This requires [Maven](https://maven.apache.org/install.html).  Typically `brew
install maven` on OS X, `apt-get install maven` on Ubuntu, or [detailed
instructions](https://maven.apache.org/install.html) otherwise.

## Example

Example function to extract the maximum polygons from a StarDist model output:
```java
private static List<Pair<Polygon2D, Double>> getMaxStarROIs(final RandomAccessibleInterval<FloatType> raysRAI,
    final RandomAccessibleInterval<FloatType> scoresRAI, final double scoreThreshold,
    final double overlapThreshold) {
    final int nRays = (int)raysRAI.dimension(2);
    final RandomAccessibleInterval<? extends Composite<FloatType>> tensor = Views.collapse(raysRAI);
    final Cursor<? extends Composite<FloatType>> raysCursor = Views.iterable(tensor).localizingCursor();

    final RandomAccess<FloatType> scoresRA = scoresRAI.randomAccess();

    final List<Pair<Polygon2D, Double>> rois = new LinkedList<>();

    // Loop over the image
    while (raysCursor.hasNext()) {
        raysCursor.fwd();
        scoresRA.setPosition(raysCursor);
        final double score = scoresRA.get().getRealDouble();

        // It the score is high enough: Add the ROI to the list
        if (score >= scoreThreshold) {
    	final Composite<FloatType> pixel = raysCursor.get();
    	final List<Double> rays = new ArrayList<>(nRays);
    	for (int i = 0; i < nRays; i++) {
    	    rays.add(pixel.get(i).getRealDouble());
    	}
    	rois.add(new ValuePair<>(new DefaultUniformStarShapedPolygon(new Point(raysCursor), rays), score));
        }
    }
    return NonMaximumSuppression.nonMaximumSuppressionOverlap(rois, overlapThreshold);
}
```

## TODOs

* Add unit tests
* Add imagej-ops for computing the interception of polygons

