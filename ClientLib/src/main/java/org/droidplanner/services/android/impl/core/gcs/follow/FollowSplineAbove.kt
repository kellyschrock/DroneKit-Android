package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location

/**
 * Created by fhuya on 1/5/15.
 */
class FollowSplineAbove(droneManager: MavLinkDroneManager, handler: Handler) :
    FollowAlgorithm(droneManager, handler) {
    private val drone: MavLinkDrone? = droneManager.drone

    public override fun processNewLocation(location: Location) {
        location.coord ?: return

        val gcsLoc = LatLong(location.coord)

        val speed = location.speed
        val bearing = location.bearing
        val bearingInRad = Math.toRadians(bearing.toDouble())
        val xVel = speed * Math.cos(bearingInRad)
        val yVel = speed * Math.sin(bearingInRad)
        drone!!.guidedPoint!!.newGuidedCoordAndVelocity(gcsLoc, xVel, yVel, 0.0)
    }

    override val type: FollowModes = FollowModes.SPLINE_ABOVE
}
