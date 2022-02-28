package org.droidplanner.services.android.impl.core.polygon

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getArea
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineLatLong
import org.droidplanner.services.android.impl.core.helpers.units.Area
import java.util.*

class Polygon {
    val points: MutableList<LatLong> = ArrayList()

    fun addPoints(pointList: List<LatLong>) {
        for (point in pointList) {
            addPoint(point)
        }
    }

    fun addPoint(coord: LatLong) {
        points.add(coord)
    }

    fun clearPolygon() {
        points.clear()
    }

    val lines: List<LineLatLong>
        get() {
            val list: MutableList<LineLatLong> = ArrayList()
            for (i in points.indices) {
                val endIndex = if (i == 0) points.size - 1 else i - 1
                list.add(LineLatLong(points[i]!!, points[endIndex]!!))
            }
            return list
        }

    fun movePoint(coord: LatLong?, number: Int) {
        points[number]!!.set(coord!!)
    }

    val area: Area
        get() = getArea(this)

    /*
	 * @Override public List<LatLng> getPathPoints() { List<LatLng> path =
	 * getLatLngList(); if (getLatLngList().size() > 2) { path.add(path.get(0));
	 * } return path; }
	 */
    @Throws(Exception::class)
    fun checkIfValid() {
        if (points.size < 3) {
            throw InvalidPolygon(points.size)
        }
    }

    inner class InvalidPolygon(var size: Int) : Exception() {
    }

    fun reversePoints() {
        points.reverse()
    }
}
