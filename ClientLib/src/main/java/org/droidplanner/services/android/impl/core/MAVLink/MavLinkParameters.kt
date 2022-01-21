package org.droidplanner.services.android.impl.core.MAVLink

import com.MAVLink.common.msg_param_request_list
import com.MAVLink.common.msg_param_request_read
import com.MAVLink.common.msg_param_set
import com.o3dr.services.android.lib.drone.property.Parameter
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import timber.log.Timber

object MavLinkParameters {
    fun requestParametersList(drone: MavLinkDrone?) {
        drone ?: return

        val msg = msg_param_request_list()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }

    fun readParameter(drone: MavLinkDrone?, name: String?) {
        drone ?: return

        val msg = msg_param_request_read()
        msg.param_index = -1
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.param_Id = name
        drone.mavClient?.sendMessage(msg, null)
    }

    fun readParameter(drone: MavLinkDrone?, index: Int) {
        drone ?: return

        val msg = msg_param_request_read()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.param_index = index.toShort()
        drone.mavClient?.sendMessage(msg, null)
    }

    fun sendParameter(drone: MavLinkDrone?, parameter: Parameter?) {
        drone ?: return
        parameter ?: return

        sendParameter(drone, parameter.name, parameter.type, parameter.value.toFloat())
    }

    fun sendParameter(drone: MavLinkDrone?, name: String?, type: Int, value: Float) {
        drone ?: return

        Timber.d("sendParameter(%s, %d, %.6f)", name, type, value)
        val msg = msg_param_set()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.param_Id = name
        msg.param_type = type.toShort()
        msg.param_value = value
        drone.mavClient?.sendMessage(msg, null)
    }
}
