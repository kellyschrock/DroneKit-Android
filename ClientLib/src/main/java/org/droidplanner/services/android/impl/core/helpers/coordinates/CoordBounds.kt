package org.droidplanner.services.android.impl.core.helpers.coordinates

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools

/**
 * Calculate a rectangle that bounds all inserted points
 */
class CoordBounds {
    @kotlin.jvm.JvmField
	var sw_3quadrant: LatLong? = null
    @kotlin.jvm.JvmField
	var ne_1quadrant: LatLong? = null

    constructor(point: LatLong) {
        include(point)
    }

    constructor(points: List<LatLong>) {
        for (point in points) {
            include(point)
        }
    }

    fun include(point: LatLong) {
        if (sw_3quadrant == null || ne_1quadrant == null) {
            ne_1quadrant = LatLong(point)
            sw_3quadrant = LatLong(point)
        } else {
            if (point.longitude > ne_1quadrant!!.longitude) {
                ne_1quadrant!!.longitude = point.longitude
            }
            if (point.latitude > ne_1quadrant!!.latitude) {
                ne_1quadrant!!.latitude = point.latitude
            }
            if (point.longitude < sw_3quadrant!!.longitude) {
                sw_3quadrant!!.longitude = point.longitude
            }
            if (point.latitude < sw_3quadrant!!.latitude) {
                sw_3quadrant!!.latitude = point.latitude
            }
        }
    }

    val diag: Double
        get() = GeoTools.latToMeters(GeoTools.getAproximatedDistance(ne_1quadrant!!, sw_3quadrant!!))

    val middle: LatLong
        get() = LatLong((ne_1quadrant!!.latitude + sw_3quadrant!!.latitude) / 2,
                (ne_1quadrant!!.longitude + sw_3quadrant!!.longitude) / 2)
}
