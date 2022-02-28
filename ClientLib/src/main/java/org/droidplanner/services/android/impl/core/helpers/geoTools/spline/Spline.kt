package org.droidplanner.services.android.impl.core.helpers.geoTools.spline

import com.o3dr.services.android.lib.coordinate.LatLong
import java.util.*

class Spline(pMinus1: LatLong?, private val p0: LatLong, p1: LatLong, p2: LatLong) {
    // derivative at a point is based on difference of previous and next
    // points
    private val p0_prime: LatLong = p1.subtract(pMinus1!!).dot(1 / SPLINE_TENSION)
    private val p1_prime = p2.subtract(p0).dot(1 / SPLINE_TENSION)
    // compute a and b coords used in spline formula
    private val a: LatLong = LatLong.sum(p0.dot(2.0), p1.dot(-2.0), p0_prime, p1_prime)
    private val b: LatLong = LatLong.sum(p0.dot(-3.0), p1.dot(3.0), p0_prime.dot(-2.0), p1_prime.negate())

    fun generateCoordinates(decimation: Int): List<LatLong> {
        val result = ArrayList<LatLong>()
        val step = 1f / decimation
        var i = 0f
        while (i < 1) {
            result.add(evaluate(i.toDouble()))
            i += step
        }
        return result
    }

    private fun evaluate(t: Double): LatLong {
        val tSquared = t * t
        val tCubed = tSquared * t
        return LatLong.sum(a.dot(tCubed), b.dot(tSquared), p0_prime.dot(t), p0)
    }

    companion object {
        private const val SPLINE_TENSION = 1.6
    }
}
