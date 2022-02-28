package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.property.Gps
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getHeadingFromCoordinates
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance

class FollowLeash(droneMgr: MavLinkDroneManager?, handler: Handler, radius: Double) :
    FollowWithRadiusAlgorithm(droneMgr!!, handler, radius) {

    override val type: FollowModes = FollowModes.LEASH

    override fun processNewLocation(location: Location) {
        val locationCoord: LatLong? = location.coord
        val droneGps = drone?.getAttribute(AttributeType.GPS) as Gps?
        val dronePosition = droneGps!!.position
        if (locationCoord == null || dronePosition == null) {
            return
        }

        if (getDistance(locationCoord, dronePosition) > radius) {
            val headingGCStoDrone = getHeadingFromCoordinates(locationCoord, dronePosition)
            val goCoord = newCoordFromBearingAndDistance(locationCoord, headingGCStoDrone, radius)
            drone?.guidedPoint?.newGuidedCoord(goCoord)
        }
    }
}
