package org.droidplanner.services.android.impl.core.drone.variables

import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

class MissionStats(myDrone: MavLinkDrone?) : DroneVariable<MavLinkDrone?>(myDrone) {
    var distanceToWP = 0.0
        private set
    var currentWP = -1
        private set
    var lastReachedWP = -1
        private set

    fun setDistanceToWp(disttowp: Double) {
        distanceToWP = disttowp
    }

    fun setWpno(seq: Int) {
        if (seq != currentWP) {
            currentWP = seq
            myDrone?.notifyDroneEvent(DroneEventsType.MISSION_WP_UPDATE)
        }
    }

    fun setLastReachedWaypointNumber(seq: Int) {
        if (seq != lastReachedWP) {
            lastReachedWP = seq
            myDrone?.notifyDroneEvent(DroneEventsType.MISSION_WP_REACHED)
        }
    }
}
