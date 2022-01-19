package org.droidplanner.services.android.impl.core.drone.autopilot.apm.variables

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduPilot
import org.droidplanner.services.android.impl.core.drone.variables.HeartBeat

class APMHeartBeat(myDrone: ArduPilot?, handler: Handler?) : HeartBeat(myDrone, handler) {
    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone) {
        when (event) {
            DroneEventsType.CALIBRATION_IMU -> {
                //Set the heartbeat in imu calibration mode.
                heartbeatState = IMU_CALIBRATION
                restartWatchdog(HEARTBEAT_IMU_CALIBRATION_TIMEOUT)
            }
            else -> super.onDroneEvent(event, drone)
        }
    }

    override fun onHeartbeatTimeout() {
        when (heartbeatState) {
            IMU_CALIBRATION -> {
                restartWatchdog(HEARTBEAT_IMU_CALIBRATION_TIMEOUT)
                myDrone.notifyDroneEvent(DroneEventsType.CALIBRATION_TIMEOUT)
            }
            else -> super.onHeartbeatTimeout()
        }
    }

    companion object {
        private const val HEARTBEAT_IMU_CALIBRATION_TIMEOUT = 35000L //ms
        protected const val IMU_CALIBRATION = 3
    }
}
