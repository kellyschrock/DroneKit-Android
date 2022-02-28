package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getAproximatedDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getHeadingFromCoordinates

class LineLatLong(val start: LatLong, val end: LatLong) {

    constructor(line: LineLatLong) : this(line.start, line.end) {}

    val heading: Double
        get() = getHeadingFromCoordinates(start, end)

    fun getFarthestEndpointTo(point: LatLong?): LatLong {
        return if (getClosestEndpointTo(point).equals(start)) {
            end
        } else {
            start
        }
    }

    fun getClosestEndpointTo(point: LatLong?): LatLong {
        return if (getDistanceToStart(point) < getDistanceToEnd(point)) {
            start
        } else {
            end
        }
    }

    private fun getDistanceToEnd(point: LatLong?): Double {
        return getAproximatedDistance(end, point!!)
    }

    private fun getDistanceToStart(point: LatLong?): Double {
        return getAproximatedDistance(start, point!!)
    }

    override fun toString(): String {
        return "from:" + start.toString() + "to:" + end.toString()
    }
}
