package org.droidplanner.services.android.impl.core.drone.variables

import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

class Type(myDrone: MavLinkDrone) : DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {

    var type = DEFAULT_TYPE
        set(type) {
            if (this.type != type) {
                field = type
                myDrone?.notifyDroneEvent(DroneEventsType.TYPE)
            }
        }

    var firmwareVersion: String? = null
        private set

    fun setFirmwareVersion(message: String) {
        if (firmwareVersion == null || firmwareVersion != message) {
            firmwareVersion = message
            myDrone?.notifyDroneEvent(DroneEventsType.FIRMWARE)
        }
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.DISCONNECTED -> type = DEFAULT_TYPE
        }
    }

    companion object {
        private const val DEFAULT_TYPE = MAV_TYPE.MAV_TYPE_GENERIC

        @JvmStatic
        fun isCopter(type: Int): Boolean {
            return when (type) {
                MAV_TYPE.MAV_TYPE_TRICOPTER,
                MAV_TYPE.MAV_TYPE_QUADROTOR,
                MAV_TYPE.MAV_TYPE_HEXAROTOR,
                MAV_TYPE.MAV_TYPE_OCTOROTOR,
                MAV_TYPE.MAV_TYPE_HELICOPTER -> true
                else -> false
            }
        }

        @JvmStatic
        fun isVtol(type: Int): Boolean {
            return when (type) {
                MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR,
                MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR,
                MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR,
                MAV_TYPE.MAV_TYPE_VTOL_RESERVED2,
                MAV_TYPE.MAV_TYPE_VTOL_RESERVED3,
                MAV_TYPE.MAV_TYPE_VTOL_RESERVED4,
                MAV_TYPE.MAV_TYPE_VTOL_RESERVED5 -> true
                else -> false
            }
        }

        @JvmStatic
        fun isPlane(type: Int): Boolean {
            return type == MAV_TYPE.MAV_TYPE_FIXED_WING
        }

        @JvmStatic
        fun isRover(type: Int): Boolean {
            return type == MAV_TYPE.MAV_TYPE_GROUND_ROVER
        }

        @JvmStatic
        fun isVehicle(type: Int): Boolean {
            return isCopter(type) || isPlane(type) || isRover(type) || isVtol(type)
        }
    }

    init {
        myDrone.addDroneListener(this)
    }
}
