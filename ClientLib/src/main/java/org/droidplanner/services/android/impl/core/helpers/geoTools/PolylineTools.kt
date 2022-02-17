package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong

object PolylineTools {
    /**
     * Total length of the polyline in meters
     *
     * @param gridPoints
     * @return
     */
    @JvmStatic
    fun getPolylineLength(gridPoints: List<LatLong>): Double {
        var length = 0.0
        for (i in 1 until gridPoints.size) {
            val to = gridPoints[i - 1] ?: continue
            length += GeoTools.getDistance(gridPoints[i], to)
        }
        return length
    }
}
