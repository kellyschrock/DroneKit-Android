package org.droidplanner.services.android.impl.core.helpers.geoTools

import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getHeadingFromCoordinates
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools
import java.util.ArrayList

class LineSampler {
    private var points: MutableList<LatLong>
    private val sampledPoints: MutableList<LatLong> = ArrayList()

    constructor(points: MutableList<LatLong>) {
        this.points = points
    }

    constructor(p1: LatLong, p2: LatLong) {
        points = ArrayList()
        points.add(p1)
        points.add(p2)
    }

    fun sample(sampleDistance: Double): List<LatLong> {
        for (i in 1 until points.size) {
            val from = points[i - 1] ?: continue
            val to = points[i]
            sampledPoints.addAll(sampleLine(from, to, sampleDistance))
        }
        val lastPoint = getLast(points)
        if (lastPoint != null) {
            sampledPoints.add(lastPoint)
        }
        return sampledPoints
    }

    private fun sampleLine(from: LatLong, to: LatLong, samplingDistance: Double): List<LatLong> {
        val result: MutableList<LatLong> = ArrayList()
        val heading = getHeadingFromCoordinates(from, to)
        val totalLength = getDistance(from, to)
        var distance = 0.0
        while (distance < totalLength) {
            result.add(newCoordFromBearingAndDistance(from, heading, distance))
            distance += samplingDistance
        }
        return result
    }

    private fun getLast(list: List<LatLong>): LatLong {
        return list[list.size - 1]
    }
}
