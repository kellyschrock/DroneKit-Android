package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.property.Gps
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getHeadingFromCoordinates
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance

/**
 * Created by fhuya on 1/5/15.
 */
class FollowSplineLeash(droneMgr: MavLinkDroneManager, handler: Handler, length: Double) :
    FollowWithRadiusAlgorithm(droneMgr!!, handler, length) {

    public override fun processNewLocation(location: Location) {
        val userLoc = location.coord
        val droneGps = drone?.getAttribute(AttributeType.GPS) as Gps?
        val droneLoc = droneGps!!.position
        if (userLoc == null || droneLoc == null) return
        if (getDistance(userLoc, droneLoc) > radius) {
            val headingGCSToDrone = getHeadingFromCoordinates(userLoc, droneLoc)
            val goCoord = newCoordFromBearingAndDistance(userLoc, headingGCSToDrone, radius)

            val speed = location.speed
            val bearing = location.bearing
            val bearingInRad = Math.toRadians(bearing.toDouble())

            val xVel = speed * Math.cos(bearingInRad)
            val yVel = speed * Math.sin(bearingInRad)
            drone?.guidedPoint?.newGuidedCoordAndVelocity(goCoord, xVel, yVel, 0.0)
        }
    }

    override val type: FollowModes = FollowModes.SPLINE_LEASH
}
