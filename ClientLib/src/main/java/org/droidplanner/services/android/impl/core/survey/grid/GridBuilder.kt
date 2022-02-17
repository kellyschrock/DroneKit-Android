package org.droidplanner.services.android.impl.core.survey.grid

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.polygon.Polygon
import org.droidplanner.services.android.impl.core.survey.SurveyData

class GridBuilder {
    private var poly: Polygon
    private var angle: Double?
    private var lineDist: Double
    private var origin: LatLong
    private var wpDistance: Double
    private var grid: Grid? = null

    constructor(polygon: Polygon, surveyData: SurveyData, originPoint: LatLong) {
        poly = polygon
        origin = originPoint
        angle = surveyData.angle
        lineDist = surveyData.lateralPictureDistance
        wpDistance = surveyData.longitudinalPictureDistance
    }

    constructor(polygon: Polygon, angle: Double, distance: Double, originPoint: LatLong) {
        poly = polygon
        origin = originPoint
        this.angle = angle
        lineDist = distance
        wpDistance = distance
    }

    fun setAngle(newAngle: Double) {
        angle = newAngle
    }

    @Throws(Exception::class)
    fun generate(sort: Boolean): Grid {
        val polygonPoints: List<LatLong> = poly.points
        val circumscribedGrid = CircumscribedGrid(polygonPoints, angle!!, lineDist).grid
        val trimmed = Trimmer(circumscribedGrid, poly.lines).mutableTrimmedGrid
        val gridSorter = EndpointSorter(trimmed, wpDistance)
        gridSorter.sortGrid(origin, sort)
        grid = Grid(gridSorter.sortedGrid, gridSorter.cameraLocations)
        return grid!!
    }
}
