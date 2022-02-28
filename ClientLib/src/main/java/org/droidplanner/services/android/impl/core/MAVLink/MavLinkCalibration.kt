package org.droidplanner.services.android.impl.core.MAVLink

import com.MAVLink.common.msg_command_ack
import com.MAVLink.common.msg_command_long
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_CMD_ACK
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

object MavLinkCalibration {
    fun sendCalibrationAckMessage(drone: MavLinkDrone?, count: Int) {
        drone ?: return
        
        val msg = msg_command_ack().apply {
            command = count
            result = MAV_CMD_ACK.MAV_CMD_ACK_OK.toShort()
        }
        
        drone.mavClient?.sendMessage(msg, null)
    }

    fun startAccelerometerCalibration(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION
            param1 = 0f
            param2 = 0f
            param3 = 0f
            param4 = 0f
            param5 = 1f
            param6 = 0f
            param7 = 0f
            confirmation = 0
        }
        
        drone.mavClient?.sendMessage(msg, listener)
    }

    /**
     * Initiate a magnetometer calibration
     * @param drone
     */
    fun startMagnetometerCalibration(drone: MavLinkDrone?, listener: ICommandListener?) {
        startMagnetometerCalibration(drone, false, false, 0, listener)
    }

    /**
     * Initiate a magnetometer calibration
     * @param drone vehicle to calibrate
     * @param retryOnFailure if true, automatically retry the magnetometer calibration if it fails
     * @param saveAutomatically if true, save the calibration automatically without user input.
     * @param startDelay positive delay in seconds before starting the calibration
     */
    fun startMagnetometerCalibration(drone: MavLinkDrone?, retryOnFailure: Boolean, saveAutomatically: Boolean,
                                     startDelay: Int, listener: ICommandListener?) {
        drone ?: return

        val msg = msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_START_MAG_CAL
            param1 = 0f
            param2 = if (retryOnFailure) 1.toFloat() else 0.toFloat()
            param3 = if (saveAutomatically) 1.toFloat() else 0.toFloat()
            param4 = if (startDelay > 0) startDelay.toFloat() else 0.toFloat()
            param5 = 0f
            param6 = 0f
            param7 = 0f
        }
        drone.mavClient?.sendMessage(msg, listener)
    }

    /**
     * Cancel the running magnetometer calibration.˛
     * @param drone
     */
    fun cancelMagnetometerCalibration(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return

        val msg = msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL
            param1 = 0f
            param2 = 0f
            param3 = 0f
            param4 = 0f
            param5 = 0f
            param6 = 0f
            param7 = 0f
        }

        drone.mavClient?.sendMessage(msg, listener)
    }

    /**
     * Accept the magnetometer calibration result.
     * @param drone
     */
    fun acceptMagnetometerCalibration(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return

        val msg = msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL
            param1 = 0f
            param2 = 0f
            param3 = 0f
            param4 = 0f
            param5 = 0f
            param6 = 0f
            param7 = 0f
        }

        drone.mavClient?.sendMessage(msg, listener)
    }
}
