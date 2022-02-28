package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.coordinates.CoordBounds
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getAproximatedDistance
import java.util.*

object LineTools {
    @JvmStatic
	fun findExternalPoints(crosses: ArrayList<LatLong>): LineLatLong {
        val meanCoord: LatLong = CoordBounds(crosses).middle
        val start = PointTools.findFarthestPoint(crosses, meanCoord)
        val end = PointTools.findFarthestPoint(crosses, start!!)
        return LineLatLong(start, end!!)
    }

    /**
     * Finds the intersection of two lines http://stackoverflow.com/questions/
     * 1119451/how-to-tell-if-a-line-intersects -a-polygon-in-c
     */
	@JvmStatic
	fun findLineIntersection(first: LineLatLong, second: LineLatLong): LatLong? {
        val denom = ((first.end.latitude - first.start.latitude) * (second.end.longitude - second
                .start.longitude)
                - (first.end.longitude - first.start.longitude) * (second.end.latitude - second
                .start.latitude))
        if (denom == 0.0) {
            //Parallel lines
            return null
        }
        val numer = ((first.start.longitude - second.start.longitude) * (second.end
                .latitude - second.start.latitude)
                - (first.start.latitude - second.start.latitude) * (second.end.longitude - second
                .start.longitude))
        val r = numer / denom
        val numer2 = ((first.start.longitude - second.start.longitude) * (first.end
                .latitude - first.start.latitude)
                - (first.start.latitude - second.start.latitude) * (first.end.longitude - first
                .start.longitude))
        val s = numer2 / denom
        if (r < 0 || r > 1 || s < 0 || s > 1) {
            //No intersection
            return null
        }
        // Find intersection point
        val x = (first.start.latitude
                + r * (first.end.latitude - first.start.latitude))
        val y = (first.start.longitude
                + r * (first.end.longitude - first.start.longitude))
        return LatLong(x, y)
    }

    /**
     * Finds the line that has the start or tip closest to a point.
     *
     * @param point
     * Point to the distance will be minimized
     * @param list
     * A list of lines to search
     * @return The closest Line
     */
	@JvmStatic
	fun findClosestLineToPoint(point: LatLong?, list: List<LineLatLong>): LineLatLong {
        var answer = list[0]
        var shortest = Double.MAX_VALUE
        for (line in list) {
            val ans1 = getAproximatedDistance(point!!, line.start)
            val ans2 = getAproximatedDistance(point, line.end)
            val shorterpnt = if (ans1 < ans2) line.start else line.end
            if (shortest > getAproximatedDistance(point, shorterpnt)) {
                answer = line
                shortest = getAproximatedDistance(point, shorterpnt)
            }
        }
        return answer
    }
}
