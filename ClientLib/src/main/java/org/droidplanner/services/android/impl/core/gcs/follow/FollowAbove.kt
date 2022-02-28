package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location

open class FollowAbove(droneMgr: MavLinkDroneManager, handler: Handler)
    : FollowAlgorithm(droneMgr, handler) {

    protected val drone = droneMgr.drone

    override val type: FollowModes? = FollowModes.ABOVE

    override fun processNewLocation(location: Location) {
        location.coord ?: return

        val gcsCoord = LatLong(location.coord)
        drone?.guidedPoint?.newGuidedCoord(gcsCoord)
    }
}
