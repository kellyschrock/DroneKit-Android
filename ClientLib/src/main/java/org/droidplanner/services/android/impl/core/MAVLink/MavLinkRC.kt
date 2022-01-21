package org.droidplanner.services.android.impl.core.MAVLink

import com.MAVLink.common.msg_rc_channels_override
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

object MavLinkRC {
    fun sendRcOverrideMsg(drone: MavLinkDrone, rcOutputs: IntArray) {
        val msg = msg_rc_channels_override()
        msg.chan1_raw = rcOutputs[0]
        msg.chan2_raw = rcOutputs[1]
        msg.chan3_raw = rcOutputs[2]
        msg.chan4_raw = rcOutputs[3]
        msg.chan5_raw = rcOutputs[4]
        msg.chan6_raw = rcOutputs[5]
        msg.chan7_raw = rcOutputs[6]
        msg.chan8_raw = rcOutputs[7]
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }
}
