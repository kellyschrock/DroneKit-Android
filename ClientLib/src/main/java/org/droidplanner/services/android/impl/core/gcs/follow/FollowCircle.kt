package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.util.MathUtils.constrainAngle
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance

class FollowCircle(
    droneMgr: MavLinkDroneManager,
    handler: Handler,
    radius: Double,
    rate: Double
) : FollowWithRadiusAlgorithm(droneMgr, handler, radius) {
    private var circleStep = 2.0
    private var circleAngle = 0.0

    override val type: FollowModes = FollowModes.CIRCLE

    public override fun processNewLocation(location: Location) {
        location.coord ?: return

        val gcsCoord = LatLong(location.coord)
        val goCoord = newCoordFromBearingAndDistance(gcsCoord, circleAngle, radius)
        circleAngle = constrainAngle(circleAngle + circleStep)
        drone?.guidedPoint?.newGuidedCoord(goCoord)
    }

    init {
        circleStep = rate
    }
}
