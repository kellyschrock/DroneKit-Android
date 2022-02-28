package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.follow.FollowHeadingAngle
import org.droidplanner.services.android.impl.core.gcs.follow.FollowAlgorithm.FollowModes

class FollowLeft(droneMgr: MavLinkDroneManager?, handler: Handler, radius: Double) :
    FollowHeadingAngle(droneMgr!!, handler, radius, -90.0) {
    override val type: FollowModes = FollowModes.LEFT
}
