package org.droidplanner.services.android.impl.core.survey.grid

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineLatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineTools
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineTools.findExternalPoints
import java.util.*

class Trimmer(grid: List<LineLatLong>, polygon: List<LineLatLong>) {
    var mutableTrimmedGrid: MutableList<LineLatLong> = ArrayList()

    private fun findCrossings(polygon: List<LineLatLong>, gridLine: LineLatLong): ArrayList<LatLong> {
        val crossings = ArrayList<LatLong>()
        for (polyLine in polygon) {
            LineTools.findLineIntersection(polyLine, gridLine)?.let { intersection ->
                crossings.add(intersection)
            }
        }
        return crossings
    }

    private fun processCrossings(crosses: ArrayList<LatLong>, gridLine: LineLatLong) {
        when (crosses.size) {
            0, 1 -> {}
            2 -> mutableTrimmedGrid.add(LineLatLong(crosses[0], crosses[1]))
            else -> mutableTrimmedGrid.add(findExternalPoints(crosses))
        }
    }

    val trimmedGrid: List<LineLatLong>
        get() = mutableTrimmedGrid

    init {
        for (gridLine in grid) {
            val crosses = findCrossings(polygon, gridLine)
            processCrossings(crosses, gridLine)
        }
    }
}
