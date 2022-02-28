package org.droidplanner.services.android.impl.core.survey.grid

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineLatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineSampler
import org.droidplanner.services.android.impl.core.helpers.geoTools.LineTools
import java.util.*

class EndpointSorter(private val grid: MutableList<LineLatLong>, private val sampleDistance: Double) {
    val gridPoints: MutableList<LatLong> = ArrayList()
    val cameraLocations: MutableList<LatLong> = ArrayList()

    @Throws(Exception::class)
    fun sortGrid(lastpnt: LatLong?, sort: Boolean) {
        var lastpnt = lastpnt
        while (grid.size > 0) {
            lastpnt = if (sort) {
                val closestLine = LineTools.findClosestLineToPoint(lastpnt, grid)
                val secondWp = processOneGridLine(closestLine, lastpnt, sort)
                secondWp
            } else {
                val closestLine = grid[0]
                val secondWp = processOneGridLine(closestLine, lastpnt, sort)
                secondWp
            }
        }
    }

    @Throws(Exception::class)
    private fun processOneGridLine(closestLine: LineLatLong, lastpnt: LatLong?, sort: Boolean): LatLong {
        val firstWP: LatLong
        val secondWp: LatLong
        firstWP = closestLine.getClosestEndpointTo(lastpnt)
        secondWp = closestLine.getFarthestEndpointTo(lastpnt)
        grid.remove(closestLine)
        updateCameraLocations(firstWP, secondWp)
        gridPoints.add(firstWP)
        gridPoints.add(secondWp)
        if (cameraLocations.size > MAX_NUMBER_OF_CAMERAS) {
            throw Exception("Too many camera positions")
        }
        return secondWp
    }

    private fun updateCameraLocations(firstWP: LatLong, secondWp: LatLong) {
        val cameraLocationsOnThisStrip = LineSampler(firstWP, secondWp)
                .sample(sampleDistance)
        cameraLocations.addAll(cameraLocationsOnThisStrip)
    }

    val sortedGrid: List<LatLong>
        get() = gridPoints

    companion object {
        private const val MAX_NUMBER_OF_CAMERAS = 24000
    }
}
