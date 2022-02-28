package org.droidplanner.services.android.impl.core.MAVLink

import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_heartbeat
import com.MAVLink.enums.MAV_AUTOPILOT
import com.MAVLink.enums.MAV_COMPONENT
import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import timber.log.Timber

/**
 * Parse the received mavlink messages, and update the drone state appropriately.
 */
class MavLinkMsgHandler(private val droneMgr: MavLinkDroneManager) {
    fun receiveData(msg: MAVLinkMessage) {
        if (msg.compid != AUTOPILOT_COMPONENT_ID) {
            if (msg.compid == MAV_COMPONENT.MAV_COMP_ID_SYSTEM_CONTROL) {
                return
            }

            // Filter these out to prevent some non-vehicle component from deciding what kind of
            // vehicle this is.
            if (msg.msgid == msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT) {
//                Timber.d("heartbeat from some other component");
                return
            }
            Timber.d("Message is from component %d", msg.compid)
            //            return;
        }
        when (msg.msgid) {
            msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT -> {
                val msg_heart = msg as msg_heartbeat
                handleHeartbeat(msg_heart)
            }
            else -> {}
        }
    }

    private fun handleHeartbeat(heartbeat: msg_heartbeat) {
        when (heartbeat.autopilot.toInt()) {
            MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA -> when (heartbeat.type.toInt()) {
                MAV_TYPE.MAV_TYPE_FIXED_WING -> droneMgr.onVehicleTypeReceived(FirmwareType.ARDU_PLANE)

                MAV_TYPE.MAV_TYPE_GENERIC,
                MAV_TYPE.MAV_TYPE_QUADROTOR,
                MAV_TYPE.MAV_TYPE_COAXIAL,
                MAV_TYPE.MAV_TYPE_HELICOPTER,
                MAV_TYPE.MAV_TYPE_HEXAROTOR,
                MAV_TYPE.MAV_TYPE_OCTOROTOR,
                MAV_TYPE.MAV_TYPE_TRICOPTER -> droneMgr.onVehicleTypeReceived(FirmwareType.ARDU_COPTER)

                MAV_TYPE.MAV_TYPE_GROUND_ROVER,
                MAV_TYPE.MAV_TYPE_SURFACE_BOAT -> droneMgr.onVehicleTypeReceived(FirmwareType.ARDU_ROVER)
            }

            MAV_AUTOPILOT.MAV_AUTOPILOT_PX4 -> droneMgr.onVehicleTypeReceived(FirmwareType.PX4_NATIVE)
            MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC,
            MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL,
            MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
            MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY -> droneMgr.onVehicleTypeReceived(FirmwareType.GENERIC)

            else -> droneMgr.onVehicleTypeReceived(FirmwareType.GENERIC)
        }
    }

    companion object {
        const val AUTOPILOT_COMPONENT_ID = 1
    }
}
