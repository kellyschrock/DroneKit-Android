package org.droidplanner.services.android.impl.core.survey.grid

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.coordinates.CoordBounds
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineLatLong
import java.util.*

class CircumscribedGrid(polygonPoints: List<LatLong>, private val angle: Double, lineDist: Double) {
    var grid: MutableList<LineLatLong> = ArrayList()

    private var gridLowerLeft: LatLong? = null
    private var extrapolatedDiag = 0.0

    @Throws(GridWithTooManyLines::class)
    private fun drawGrid(lineDist: Double) {
        var lines = 0
        var startPoint = gridLowerLeft
        while (lines * lineDist < extrapolatedDiag) {
            val endPoint = newCoordFromBearingAndDistance(startPoint!!, angle,
                    extrapolatedDiag)
            val line = LineLatLong(startPoint, endPoint)
            grid.add(line)
            startPoint = newCoordFromBearingAndDistance(startPoint, angle + 90, lineDist)
            lines++
            if (lines > MAX_NUMBER_OF_LINES) {
                throw GridWithTooManyLines()
            }
        }
    }

    private fun findPolygonBounds(polygonPoints: List<LatLong>) {
        val bounds = CoordBounds(polygonPoints)
        val middlePoint = bounds.middle
        gridLowerLeft = newCoordFromBearingAndDistance(middlePoint, angle - 135,
                bounds.diag)
        extrapolatedDiag = bounds.diag * 1.5
    }

    class GridWithTooManyLines : Exception() {
    }

    companion object {
        private const val MAX_NUMBER_OF_LINES = 7200
    }

    init {
        findPolygonBounds(polygonPoints)
        drawGrid(lineDist)
    }
}
