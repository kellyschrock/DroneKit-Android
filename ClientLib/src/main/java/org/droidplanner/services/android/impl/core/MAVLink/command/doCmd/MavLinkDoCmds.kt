package org.droidplanner.services.android.impl.core.MAVLink.command.doCmd

import com.MAVLink.ardupilotmega.msg_digicam_control
import com.MAVLink.ardupilotmega.msg_mount_control
import com.MAVLink.common.msg_command_long
import com.MAVLink.common.msg_mission_set_current
import com.MAVLink.enums.GRIPPER_ACTIONS
import com.MAVLink.enums.MAV_CMD
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

object MavLinkDoCmds {
    @JvmStatic
    fun setVehicleHome(drone: MavLinkDrone?, location: LatLongAlt?, listener: ICommandListener?) {
        drone ?: return
        location ?: return
        
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_SET_HOME
            param5 = location.latitude.toFloat()
            param6 = location.longitude.toFloat()
            param7 = location.altitude.toFloat()
        }, listener)
    }

    @JvmStatic
    fun setROI(drone: MavLinkDrone?, coord: LatLongAlt?, listener: ICommandListener?) {
        drone ?: return
        coord ?: return
        
        val isClear = coord.latitude == 0.0 && coord.longitude == 0.0
        if (isClear) {
            resetROI(drone, listener)
            return
        }
        
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            // msg.command = MAV_CMD.MAV_CMD_DO_SET_ROI_LOCATION;
            // "Hurr durr, don't use this as of January 2018"
            command = MAV_CMD.MAV_CMD_DO_SET_ROI
            param5 = coord.latitude.toFloat()
            param6 = coord.longitude.toFloat()
            param7 = coord.altitude.toFloat()
        }, listener)
    }

    @JvmStatic
    fun resetROI(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_SET_ROI
        }, listener)
    }

    @JvmStatic
    fun triggerCamera(drone: MavLinkDrone?) {
        drone ?: return
        
        drone.mavClient?.sendMessage(msg_digicam_control().apply {
            target_system = drone.sysid
            target_component = drone.compid
            shot = 1
        }, null)
    }

    @JvmStatic
    fun empCommand(drone: MavLinkDrone?, release: Boolean, listener: ICommandListener?) {
        drone ?: return
        
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_GRIPPER
            param2 = if (release) GRIPPER_ACTIONS.GRIPPER_ACTION_RELEASE.toFloat() else GRIPPER_ACTIONS.GRIPPER_ACTION_GRAB.toFloat()
        }, listener)
    }

    /**
     * Set a Relay pin’s voltage high or low
     *
     * @param drone       target vehicle
     * @param relayNumber
     * @param enabled     true for relay to be on, false for relay to be off.
     */
    @JvmStatic
    fun setRelay(drone: MavLinkDrone?, relayNumber: Int, enabled: Boolean, listener: ICommandListener?) {
        drone ?: return
        
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_SET_RELAY
            param1 = relayNumber.toFloat()
            param2 = if (enabled) 1f else 0f
        }, listener)
    }

    /**
     * Move a servo to a particular pwm value
     *
     * @param drone   target vehicle
     * @param channel he output channel the servo is attached to
     * @param pwm     PWM value to output to the servo. Servo’s generally accept pwm values between 1000 and 2000
     */
    @JvmStatic
    fun setServo(drone: MavLinkDrone?, channel: Int, pwm: Int, listener: ICommandListener?) {
        drone ?: return
        
        drone.mavClient?.sendMessage(msg_command_long().apply {
            target_system = drone.sysid
            target_component = drone.compid
            command = MAV_CMD.MAV_CMD_DO_SET_SERVO
            param1 = channel.toFloat()
            param2 = pwm.toFloat()
        }, listener)
    }

    /**
     * Set the orientation of a gimbal
     *
     * @param drone    target vehicle
     * @param pitch    the desired gimbal pitch in degrees
     * @param roll     the desired gimbal roll in degrees
     * @param yaw      the desired gimbal yaw in degrees
     * @param listener Register a callback to receive update of the command execution state.
     */
    @JvmStatic
    fun setGimbalOrientation(drone: MavLinkDrone?, pitch: Float, roll: Float, yaw: Float, listener: ICommandListener?) {
        drone ?: return
        
        drone.mavClient?.sendMessage(msg_mount_control().apply {
            target_system = drone.sysid
            target_component = drone.compid
            input_a = (pitch * 100).toInt()
            input_b = (roll * 100).toInt()
            input_c = (yaw * 100).toInt()
        }, listener)
    }

    /**
     * Jump to the desired command in the mission list. Repeat this action only the specified number of times
     *
     * @param drone    target vehicle
     * @param waypoint command
     * @param listener Register a callback to receive update of the command execution state.
     */
    @JvmStatic
    fun gotoWaypoint(drone: MavLinkDrone?, waypoint: Int, listener: ICommandListener?) {
        drone ?: return

        drone.mavClient?.sendMessage(msg_mission_set_current().apply {
            seq = waypoint
        }, listener)
    }
}
