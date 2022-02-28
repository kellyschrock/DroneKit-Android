package org.droidplanner.services.android.impl.core.drone.variables

import com.MAVLink.common.msg_raw_imu
import com.o3dr.services.android.lib.drone.property.Parameter
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

class Magnetometer(myDrone: MavLinkDrone?) : DroneVariable<MavLinkDrone?>(myDrone) {
    var x = 0
        private set
    var y = 0
        private set
    var z = 0
        private set

    fun newData(msg_imu: msg_raw_imu?) {
        msg_imu ?: return

        x = msg_imu.xmag.toInt()
        y = msg_imu.ymag.toInt()
        z = msg_imu.zmag.toInt()
        myDrone?.notifyDroneEvent(DroneEventsType.MAGNETOMETER)
    }

    val vector: IntArray
        get() = intArrayOf(x, y, z)

    val offsets: IntArray?
        get() {
            myDrone?.let { drone ->
                val paramX = drone.parameterManager?.getParameter("COMPASS_OFS_X")
                val paramY = drone.parameterManager?.getParameter("COMPASS_OFS_Y")
                val paramZ = drone.parameterManager?.getParameter("COMPASS_OFS_Z")
                return if (paramX == null || paramY == null || paramZ == null) {
                    null
                } else intArrayOf(paramX.value.toInt(), paramY.value.toInt(), paramZ.value.toInt())
            } ?: run { return null }
        }
}
