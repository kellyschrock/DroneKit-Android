package org.droidplanner.services.android.impl.core.MAVLink

import com.MAVLink.common.*
import com.MAVLink.enums.MAV_MISSION_RESULT
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

object MavLinkWaypoint {
    fun sendAck(drone: MavLinkDrone?) {
        drone ?: return

        val msg = msg_mission_ack()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.type = MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED.toShort()
        msg.isMavlink2 = false
        msg.mission_type = 0
        drone.mavClient?.sendMessage(msg, null)
    }

    @JvmStatic
    fun requestWayPoint(drone: MavLinkDrone?, index: Int) {
        drone ?: return

        val msg = msg_mission_request()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.seq = index
        msg.isMavlink2 = false
        msg.mission_type = 0
        drone.mavClient?.sendMessage(msg, null)
    }

    fun requestWaypointsList(drone: MavLinkDrone?) {
        drone ?: return

        val msg = msg_mission_request_list()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.isMavlink2 = false
        msg.mission_type = 0
        drone.mavClient?.sendMessage(msg, null)
    }

    fun sendWaypointCount(drone: MavLinkDrone?, count: Int) {
        drone ?: return

        val msg = msg_mission_count()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.count = count
        msg.isMavlink2 = false
        msg.mission_type = 0
        drone.mavClient!!.sendMessage(msg, null)
    }

    fun sendSetCurrentWaypoint(drone: MavLinkDrone?, i: Short) {
        drone ?: return

        val msg = msg_mission_set_current()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.seq = i.toInt()
        drone.mavClient?.sendMessage(msg, null)
    }
}
