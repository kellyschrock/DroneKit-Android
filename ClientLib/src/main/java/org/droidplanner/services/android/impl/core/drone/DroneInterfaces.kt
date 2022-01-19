package org.droidplanner.services.android.impl.core.drone

import android.os.Bundle
import com.o3dr.services.android.lib.drone.property.Parameter
import org.droidplanner.services.android.impl.core.MAVLink.WaypointManager.WaypointEvent_Type
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone

class DroneInterfaces {
    /**
     * Sets of drone events used for broadcast throughout the app.
     */
    enum class DroneEventsType {
        ALTITUDE,
        ORIENTATION,
        SPEED,
        BATTERY,
        GUIDEDPOINT,
        ATTITUDE,
        RADIO,
        RC_IN,
        RC_OUT,
        ARMING,
        AUTOPILOT_WARNING,
        MODE,
        STATE,
        MISSION_UPDATE,
        MISSION_RECEIVED,
        TYPE,
        HOME,
        CALIBRATION_IMU,
        CALIBRATION_TIMEOUT,
        HEARTBEAT_TIMEOUT,
        HEARTBEAT_FIRST,
        HEARTBEAT_RESTORED,
        DISCONNECTED,
        CONNECTED,
        CONNECTING,
        MISSION_SENT,
        ARMING_STARTED,
        INVALID_POLYGON,
        MISSION_WP_UPDATE,
        WARNING_SIGNAL_WEAK,
        FIRMWARE,
        WARNING_NO_GPS,
        MAGNETOMETER,
        FOOTPRINT,
        EKF_STATUS_UPDATE,
        EKF_POSITION_STATE_UPDATE,
        MISSION_WP_REACHED
    }

    interface OnDroneListener<T : Drone?> {
        fun onDroneEvent(event: DroneEventsType, drone: T)
    }

    interface AttributeEventListener {
        fun onAttributeEvent(attributeEvent: String, eventInfo: Bundle)
    }

    interface OnParameterManagerListener {
        fun onBeginReceivingParameters()
        fun onParameterReceived(parameter: Parameter, index: Int, count: Int)
        fun onEndReceivingParameters()
    }

    interface OnWaypointManagerListener {
        fun onBeginWaypointEvent(wpEvent: WaypointEvent_Type)
        fun onWaypointEvent(wpEvent: WaypointEvent_Type, index: Int, count: Int)
        fun onEndWaypointEvent(wpEvent: WaypointEvent_Type)
    }
}
