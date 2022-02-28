package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance

abstract class FollowHeadingAngle protected constructor(
    droneMgr: MavLinkDroneManager,
    handler: Handler,
    radius: Double,
    protected var angleOffset: Double
) : FollowWithRadiusAlgorithm(droneMgr, handler, radius) {

    public override fun processNewLocation(location: Location) {
        location.coord ?: return

        val gcsCoord = LatLong(location.coord)
        val bearing = location.bearing
        val goCoord = newCoordFromBearingAndDistance(gcsCoord, bearing + angleOffset, radius)
        drone?.guidedPoint?.newGuidedCoord(goCoord)
    }
}
