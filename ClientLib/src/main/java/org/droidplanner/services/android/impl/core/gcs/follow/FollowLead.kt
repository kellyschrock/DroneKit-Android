package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager

class FollowLead(droneMgr: MavLinkDroneManager?, handler: Handler, radius: Double)
    : FollowHeadingAngle(droneMgr!!, handler, radius, 0.0) {

    override val type: FollowModes? = FollowModes.LEAD
}
