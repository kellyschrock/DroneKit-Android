package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.follow.FollowAlgorithm
import org.droidplanner.services.android.impl.core.gcs.follow.FollowAlgorithm.FollowModes
import org.droidplanner.services.android.impl.core.gcs.location.Location

/**
 * Created by Fredia Huya-Kouadio on 3/23/15.
 */
class FollowLookAtMe(droneMgr: MavLinkDroneManager, handler: Handler)
    : FollowAlgorithm(droneMgr, handler) {

    override fun processNewLocation(location: Location) {}
    override val type: FollowModes = FollowModes.LOOK_AT_ME
}
