package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager

/**
 * Created by Fredia Huya-Kouadio on 1/27/15.
 */
abstract class FollowWithRadiusAlgorithm(droneMgr: MavLinkDroneManager, handler: Handler,
    protected var radius: Double
) : FollowAlgorithm(droneMgr, handler) {
    protected val drone: MavLinkDrone? = droneMgr.drone

    override val params: Map<String, Any> get() {
        val params: MutableMap<String, Any> = HashMap()
        params[EXTRA_FOLLOW_RADIUS] = radius
        return params
    }

    override fun updateAlgorithmParams(params: Map<String, Any?>) {
        super.updateAlgorithmParams(params)
        (params[EXTRA_FOLLOW_RADIUS] as? Double)?.let { updatedRadius ->
            radius = Math.max(0.0, updatedRadius)
        }
    }

    companion object {
        const val EXTRA_FOLLOW_RADIUS = "extra_follow_radius"
    }
}
