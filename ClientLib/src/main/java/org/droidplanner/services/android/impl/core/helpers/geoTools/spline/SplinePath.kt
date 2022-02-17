package org.droidplanner.services.android.impl.core.helpers.geoTools.spline

import com.o3dr.services.android.lib.coordinate.LatLong
import java.util.ArrayList

/**
 * This class contains functions used to generate a spline path.
 */
object SplinePath {
    /**
     * Used as tag for logging.
     */
    private val TAG = SplinePath::class.java.simpleName
    private const val SPLINE_DECIMATION = 20

    /**
     * Process the given map coordinates, and return a set of coordinates
     * describing the spline path.
     *
     * @param points
     * map coordinates decimation factor
     * @return set of coordinates describing the spline path
     */
    @JvmStatic
    fun process(points: List<LatLong>): List<LatLong> {
        val pointsCount = points.size
        if (pointsCount < 4) {
            System.err.println("Not enough points!")
            return points
        }
        val results = processPath(points)
        results.add(0, points[0])
        results.add(points[pointsCount - 1])
        return results
    }

    private fun processPath(points: List<LatLong>): MutableList<LatLong> {
        val results: MutableList<LatLong> = ArrayList()
        for (i in 3 until points.size) {
            results.addAll(processPathSegment(points[i - 3], points[i - 2],
                    points[i - 1], points[i]))
        }
        return results
    }

    private fun processPathSegment(l1: LatLong, l2: LatLong, l3: LatLong, l4: LatLong): List<LatLong> {
        val spline = Spline(l1, l2, l3, l4)
        return spline.generateCoordinates(SPLINE_DECIMATION)
    }
}
