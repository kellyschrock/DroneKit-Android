package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getAproximatedDistance
import java.util.*

object PointTools {
    fun findFarthestPoint(crosses: ArrayList<LatLong>, middle: LatLong): LatLong? {
        var farthestDistance = Double.NEGATIVE_INFINITY
        var farthestPoint: LatLong? = null
        for (cross in crosses) {
            val distance = getAproximatedDistance(cross!!, middle!!)
            if (distance > farthestDistance) {
                farthestPoint = cross
                farthestDistance = distance
            }
        }
        return farthestPoint
    }

    /**
     * Finds the closest point in a list to another point
     *
     * @param point
     * point that will be used as reference
     * @param list
     * List of points to be searched
     * @return The closest point
     */
    private fun findClosestPoint(point: LatLong, list: List<LatLong>): LatLong? {
        var answer: LatLong? = null
        var currentbest = Double.MAX_VALUE
        for (pnt in list) {
            val dist1 = getAproximatedDistance(point, pnt)
            if (dist1 < currentbest) {
                answer = pnt
                currentbest = dist1
            }
        }
        return answer
    }

    /**
     * Finds the pair of adjacent points that minimize the distance to a
     * reference point
     *
     * @param point
     * point that will be used as reference
     * @param waypoints2
     * List of points to be searched
     * @return Position of the second point in the pair that minimizes the
     * distance
     */
    fun findClosestPair(point: LatLong, waypoints2: List<LatLong>): Int {
        var answer = 0
        var currentbest = Double.MAX_VALUE
        var dist: Double
        var p1: LatLong
        var p2: LatLong
        for (i in waypoints2.indices) {
            if (i == waypoints2.size - 1) {
                p1 = waypoints2[i]
                p2 = waypoints2[0]
            } else {
                p1 = waypoints2[i]
                p2 = waypoints2[i + 1]
            }
            dist = pointToLineDistance(p1, p2, point)
            if (dist < currentbest) {
                answer = i + 1
                currentbest = dist
            }
        }
        return answer
    }

    /**
     * Provides the distance from a point P to the line segment that passes
     * through A-B. If the point is not on the side of the line, returns the
     * distance to the closest point
     *
     * @param L1
     * First point of the line
     * @param L2
     * Second point of the line
     * @param P
     * Point to measure the distance
     */
    @JvmStatic
    fun pointToLineDistance(L1: LatLong, L2: LatLong, P: LatLong): Double {
        val A = P.latitude - L1.latitude
        val B = P.longitude - L1.longitude
        val C = L2.latitude - L1.latitude
        val D = L2.longitude - L1.longitude
        val dot = A * C + B * D
        val len_sq = C * C + D * D
        val param = dot / len_sq
        val xx: Double
        val yy: Double
        if (param < 0) // point behind the segment
        {
            xx = L1.latitude
            yy = L1.longitude
        } else if (param > 1) // point after the segment
        {
            xx = L2.latitude
            yy = L2.longitude
        } else { // point on the side of the segment
            xx = L1.latitude + param * C
            yy = L1.longitude + param * D
        }
        return Math.hypot(xx - P.latitude, yy - P.longitude)
    }
}
