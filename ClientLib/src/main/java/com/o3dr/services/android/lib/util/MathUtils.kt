package com.o3dr.services.android.lib.util

import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.util.MathUtils
import com.o3dr.services.android.lib.coordinate.LatLong
import java.util.ArrayList

/**
 * Utility functions for math.
 */
object MathUtils {
    private const val RADIUS_OF_EARTH_IN_METERS = 6378137.0 // Source: WGS84
    const val SIGNAL_MAX_FADE_MARGIN = 50
    const val SIGNAL_MIN_FADE_MARGIN = 6

    /**
     * Computes the distance between two points taking into consideration altitude.
     * @param from  start lat/long position
     * @param to    end lat/long position
     * @return      distance between positions in meters.
     */
    @JvmStatic
    fun getDistance3D(from: LatLongAlt?, to: LatLongAlt?): Double {
        if (from == null || to == null) {
            return (-1).toDouble()
        }

        val distance2d = getDistance2D(from, to)
        val distanceSqr = Math.pow(distance2d, 2.0)
        val altitudeSqr = Math.pow(to.altitude - from.altitude, 2.0)
        return Math.sqrt(altitudeSqr + distanceSqr)
    }

    /**
     * Computes the distance between two points without considering altitude.
     * @param from  start lat/long position
     * @param to    end lat/long position
     * @return      distance between positions in meters.
     */
    @JvmStatic
    fun getDistance2D(from: LatLong?, to: LatLong?): Double {
        return if (from == null || to == null) {
            (-1).toDouble()
        } else RADIUS_OF_EARTH_IN_METERS * Math.toRadians(getArcInRadians(from, to))
    }

    /**
     * Compute a new Lat/Long point (without altitude) given specified changes along latitude and
     * longitude.
     * @param from      start lat/long position
     * @param xMeters   longitude change in meters
     * @param yMeters   latitude change in meters
     * @return          new lat/long position.
     */
    @JvmStatic
    fun addDistance(from: LatLong, xMeters: Double, yMeters: Double): LatLong {
        val lat = from.latitude
        val lon = from.longitude

        // Coordinate offsets in radians
        val dLat = yMeters / RADIUS_OF_EARTH_IN_METERS
        val dLon = xMeters / (RADIUS_OF_EARTH_IN_METERS * Math.cos(Math.PI * lat / 180))

        // OffsetPosition, decimal degrees
        val latO = lat + dLat * 180 / Math.PI
        val lonO = lon + dLon * 180 / Math.PI
        return LatLong(latO, lonO)
    }

    /**
     * Calculates the arc between two points (http://en.wikipedia.org/wiki/Haversine_formula).
     * @param from  start lat/long position
     * @param to    stop lat/long position
     * @return      the arc in degrees
     */
    @JvmStatic
    fun getArcInRadians(from: LatLong, to: LatLong): Double {
        val latitudeArc = Math.toRadians(from.latitude - to.latitude)
        val longitudeArc = Math.toRadians(from.longitude - to.longitude)
        var latitudeH = Math.sin(latitudeArc * 0.5)
        latitudeH *= latitudeH
        var lontitudeH = Math.sin(longitudeArc * 0.5)
        lontitudeH *= lontitudeH
        val tmp = (Math.cos(Math.toRadians(from.latitude))
                * Math.cos(Math.toRadians(to.latitude)))
        return Math.toDegrees(2.0 * Math.asin(Math.sqrt(latitudeH + tmp * lontitudeH)))
    }

    /**
     * Signal strength in percentage.
     * @param fadeMargin    TODO
     * @param remFadeMargin TODO
     * @return percentage   TODO
     */
    @JvmStatic
    fun getSignalStrength(fadeMargin: Double, remFadeMargin: Double): Int {
        return (normalize(Math.min(fadeMargin, remFadeMargin),
                SIGNAL_MIN_FADE_MARGIN.toDouble(), SIGNAL_MAX_FADE_MARGIN.toDouble()) * 100).toInt()
    }

    /**
     * TODO
     * @param value TODO
     * @param min   TODO
     * @param max   TODO
     * @return      TODO
     */
    @JvmStatic
    fun normalize(value: Double, min: Double, max: Double): Double {
        var value = value
        value = constrain(value, min, max)
        return (value - min) / (max - min)
    }

    private fun constrain(value: Double, min: Double, max: Double): Double {
        var value = value
        value = Math.max(value, min)
        value = Math.min(value, max)
        return value
    }

    /**
     * Compute the difference between two angles.
     * @param a     Minuend angle in degrees
     * @param b     Subtrahend angle in degrees.
     * @return      Difference between the angles in degrees
     */
    @JvmStatic
    fun angleDiff(a: Double, b: Double): Double {
        var diff = Math.IEEEremainder(b - a + 180, 360.0)
        if (diff < 0) diff += 360.0
        return diff - 180
    }

    /**
     * TODO
     * @param x TODO
     * @return  TODO
     */
    @JvmStatic
    fun constrainAngle(x: Double): Double {
        var x = x
        x = Math.IEEEremainder(x, 360.0)
        if (x < 0) x += 360.0
        return x
    }

    /**
     * TODO
     * @param a     TODO
     * @param b     TODO
     * @param alpha TODO
     * @return      TODO
     */
    @JvmStatic
    fun bisectAngle(a: Double, b: Double, alpha: Double): Double {
        return constrainAngle(a + angleDiff(a, b) * alpha)
    }

    /**
     * TODO
     * @param altDelta  TODO
     * @param distDelta TODO
     * @return          TODO
     */
    @JvmStatic
    fun hypot(altDelta: Double, distDelta: Double): Double {
        return Math.hypot(altDelta, distDelta)
    }

    /**
     * Create a rotation matrix given some euler angles this is based on
     * http://gentlenav.googlecode.com/files/EulerAngles.pdf
     * @param roll  vehicle roll in degrees
     * @param pitch vehicle pitch in degrees
     * @param yaw   vehicle yaw in degrees
     * @return      Rotation matrix
     */
    @JvmStatic
    fun dcmFromEuler(roll: Double, pitch: Double, yaw: Double): Array<DoubleArray> {
        val dcm = Array(3) { DoubleArray(3) }
        val cp = Math.cos(pitch)
        val sp = Math.sin(pitch)
        val sr = Math.sin(roll)
        val cr = Math.cos(roll)
        val sy = Math.sin(yaw)
        val cy = Math.cos(yaw)
        dcm[0][0] = cp * cy
        dcm[1][0] = sr * sp * cy - cr * sy
        dcm[2][0] = cr * sp * cy + sr * sy
        dcm[0][1] = cp * sy
        dcm[1][1] = sr * sp * sy + cr * cy
        dcm[2][1] = cr * sp * sy - sr * cy
        dcm[0][2] = -sp
        dcm[1][2] = sr * cp
        dcm[2][2] = cr * cp
        return dcm
    }

    /**
     * Based on the Ramer–Douglas–Peucker algorithm
     * http://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
     * @param list      List of lat/long points in the curve.
     * @param epsilon   Tolerance for determining list of points for approximation of curve.
     * @return          List of lat/long points in the approximated curve.
     */
    @JvmStatic
    fun simplify(list: List<LatLong>, epsilon: Double): List<LatLong> {
        var index = 0
        var dmax = 0.0
        val lastIndex = list.size - 1

        // Find the point with the maximum distance.
        for (i in 1 until lastIndex) {
            val d = pointToLineDistance(list[0], list[lastIndex], list[i])
            if (d > dmax) {
                index = i
                dmax = d
            }
        }

        // If max distance is greater than epsilon, recursively simplify.
        val ResultList: MutableList<LatLong> = ArrayList()
        if (dmax > epsilon) {
            // Recursive call.
            val recResults1 = mutableListOf<LatLong>()
            recResults1.addAll(simplify(list.subList(0, index + 1), epsilon))
            val recResults2 = simplify(list.subList(index, lastIndex + 1), epsilon)

            // Build the result list.
            recResults1.removeAt(recResults1.size - 1)
            ResultList.addAll(recResults1)
            ResultList.addAll(recResults2)
        } else {
            ResultList.add(list[0])
            ResultList.add(list[lastIndex])
        }
        return ResultList
    }

    /**
     * Provides the distance from a point P to the line segment that passes
     * through A-B. If the point is not on the side of the line, returns the
     * distance to the closest point
     *
     * @param L1    First point of the line
     * @param L2    Second point of the line
     * @param P     Point to measure the distance
     * @return      distance between point and line in meters.
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

    /**
     * Computes the heading between two coordinates.
     * @param fromLoc   start lat/long position
     * @param toLoc     end lat/long position
     * @return          heading in degrees
     */
    @JvmStatic
    fun getHeadingFromCoordinates(fromLoc: LatLong, toLoc: LatLong): Double {
        val fLat = Math.toRadians(fromLoc.latitude)
        val fLng = Math.toRadians(fromLoc.longitude)
        val tLat = Math.toRadians(toLoc.latitude)
        val tLng = Math.toRadians(toLoc.longitude)
        val degree = Math.toDegrees(Math.atan2(
                Math.sin(tLng - fLng) * Math.cos(tLat),
                Math.cos(fLat) * Math.sin(tLat) - (Math.sin(fLat) * Math.cos(tLat)
                        * Math.cos(tLng - fLng))))
        return if (degree >= 0) {
            degree
        } else {
            360 + degree
        }
    }

    /**
     * Extrapolate latitude/longitude given a heading and distance thanks to
     * http://www.movable-type.co.uk/scripts/latlong.html
     *
     * @param origin    Point of origin
     * @param bearing   bearing to navigate
     * @param distance  distance to be added
     * @return          new point with the added distance
     */
    @JvmStatic
    fun newCoordFromBearingAndDistance(origin: LatLong, bearing: Double,
                                       distance: Double): LatLong {
        val lat = origin.latitude
        val lon = origin.longitude
        val lat1 = Math.toRadians(lat)
        val lon1 = Math.toRadians(lon)
        val brng = Math.toRadians(bearing)
        val dr = distance / RADIUS_OF_EARTH_IN_METERS
        val lat2 = Math.asin(Math.sin(lat1) * Math.cos(dr) + (Math.cos(lat1) * Math.sin(dr)
                * Math.cos(brng)))
        val lon2 = (lon1
                + Math.atan2(Math.sin(brng) * Math.sin(dr) * Math.cos(lat1),
                Math.cos(dr) - Math.sin(lat1) * Math.sin(lat2)))
        return LatLong(Math.toDegrees(lat2), Math.toDegrees(lon2))
    }

    /**
     * Compute total length of the polyline in meters.
     *
     * @param gridPoints    list of lat/long points for the polyline.
     * @return              length of the polyline in meters.
     */
    @JvmStatic
    fun getPolylineLength(gridPoints: List<LatLong?>): Double {
        var length = 0.0
        for (i in 1 until gridPoints.size) {
            val to = gridPoints[i - 1] ?: continue
            length += getDistance2D(gridPoints[i], to)
        }
        return length
    }

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
         * @param points map coordinates decimation factor
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

    class Spline(pMinus1: LatLong, private val p0: LatLong, p1: LatLong, p2: LatLong) {
        private val p0_prime: LatLong = p1.subtract(pMinus1).dot((1 / SPLINE_TENSION).toDouble())
        private val a: LatLong
        private val b: LatLong
        fun generateCoordinates(decimation: Int): List<LatLong> {
            val result = ArrayList<LatLong>()
            val step = 1f / decimation
            var i = 0f
            while (i < 1) {
                result.add(evaluate(i))
                i += step
            }
            return result
        }

        private fun evaluate(t: Float): LatLong {
            val tSquared = t * t
            val tCubed = tSquared * t
            return LatLong.sum(a.dot(tCubed.toDouble()), b.dot(tSquared.toDouble()), p0_prime.dot(t.toDouble()), p0)
        }

        companion object {
            private const val SPLINE_TENSION = 1.6f
        }

        init {

            // derivative at a point is based on difference of previous and next
            // points
            val p1_prime = p2.subtract(p0).dot((1 / SPLINE_TENSION).toDouble())

            // compute a and b coords used in spline formula
            a = LatLong.sum(p0.dot(2.0), p1.dot(-2.0), p0_prime, p1_prime)
            b = LatLong.sum(p0.dot(-3.0), p1.dot(3.0), p0_prime.dot(-2.0), p1_prime.negate())
        }
    }
}
