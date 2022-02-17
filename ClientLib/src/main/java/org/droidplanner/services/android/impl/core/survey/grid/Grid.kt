package org.droidplanner.services.android.impl.core.survey.grid

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.PolylineTools

class Grid(var gridPoints: List<LatLong>, val cameraLocations: List<LatLong>) {
    val length: Double
        get() = PolylineTools.getPolylineLength(gridPoints)
    val numberOfLines: Int
        get() = gridPoints.size / 2

    fun getCameraCount(): Int {
        return cameraLocations.size
    }
}
