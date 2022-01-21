package org.droidplanner.services.android.impl.core.drone.variables

import com.MAVLink.common.msg_rc_channels_raw
import com.MAVLink.common.msg_servo_output_raw
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType

class RC(myDrone: MavLinkDrone?) : DroneVariable<MavLinkDrone?>(myDrone) {
    var input = IntArray(8)
    var out = IntArray(8)

    fun setRcInputValues(msg: msg_rc_channels_raw?) {
        msg ?: return

        input[0] = msg.chan1_raw
        input[1] = msg.chan2_raw
        input[2] = msg.chan3_raw
        input[3] = msg.chan4_raw
        input[4] = msg.chan5_raw
        input[5] = msg.chan6_raw
        input[6] = msg.chan7_raw
        input[7] = msg.chan8_raw
        myDrone?.notifyDroneEvent(DroneEventsType.RC_IN)
    }

    fun setRcOutputValues(msg: msg_servo_output_raw?) {
        msg ?: return

        out[0] = msg.servo1_raw
        out[1] = msg.servo2_raw
        out[2] = msg.servo3_raw
        out[3] = msg.servo4_raw
        out[4] = msg.servo5_raw
        out[5] = msg.servo6_raw
        out[6] = msg.servo7_raw
        out[7] = msg.servo8_raw
        myDrone?.notifyDroneEvent(DroneEventsType.RC_OUT)
    }
}
