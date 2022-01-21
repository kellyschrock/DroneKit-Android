package org.droidplanner.services.android.impl.core.MAVLink

import android.util.Log
import com.MAVLink.common.*
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import com.MAVLink.enums.MAV_GOTO
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition.TargetState
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import timber.log.Timber

object MavLinkCommands {
    val TAG = MavLinkCommands::class.java.simpleName
    
    private const val EMERGENCY_DISARM_MAGIC_NUMBER = 21196
    
    private const val MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = 1 shl 0 or (1 shl 1) or (1 shl 2)
    private const val MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = 1 shl 3 or (1 shl 4) or (1 shl 5)
    private const val MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = 1 shl 6 or (1 shl 7) or (1 shl 8)
    
    fun sendVTOLTransition(drone: MavLinkDrone?, state: TargetState, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION
        msg.param1 = state.state.toFloat()
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun changeMissionSpeed(drone: MavLinkDrone?, speed: Float, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_DO_CHANGE_SPEED
        msg.param1 = 0f // TODO use correct parameter
        msg.param2 = speed
        msg.param3 = 0f // TODO use correct parameter
        drone.mavClient?.sendMessage(msg, listener)
    }

    fun setGuidedMode(drone: MavLinkDrone?, latitude: Double, longitude: Double, d: Double) {
        Timber.d("setGuidedMode(): lat=%.4f lng=%.4f alt=%.1f", latitude, longitude, d)
        drone ?: return
        
        val msg = msg_mission_item()
        msg.seq = 0
        msg.current = 2 // TODO use guided mode enum
        msg.frame = MAV_FRAME.MAV_FRAME_GLOBAL.toShort()
        msg.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT //
        msg.param1 = 0f // TODO use correct parameter
        msg.param2 = 0f // TODO use correct parameter
        msg.param3 = 0f // TODO use correct parameter
        msg.param4 = 0f // TODO use correct parameter
        msg.x = latitude.toFloat()
        msg.y = longitude.toFloat()
        msg.z = d.toFloat()
        msg.autocontinue = 1 // TODO use correct parameter
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }

    fun sendGuidedPosition(drone: MavLinkDrone?, latitude: Double, longitude: Double, altitude: Double) {
        drone ?: return
        
        val msg = msg_set_position_target_global_int()
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE or MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.toShort()
        msg.lat_int = (latitude * 1E7).toInt()
        msg.lon_int = (longitude * 1E7).toInt()
        msg.alt = altitude.toFloat()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }

    fun sendGuidedVelocity(drone: MavLinkDrone?, xVel: Double, yVel: Double, zVel: Double) {
        drone ?: return
        
        val msg = msg_set_position_target_global_int()
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE or MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.toShort()
        msg.vx = xVel.toFloat()
        msg.vy = yVel.toFloat()
        msg.vz = zVel.toFloat()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }

    fun setVelocityInLocalFrame(drone: MavLinkDrone?, xVel: Float, yVel: Float, zVel: Float, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_set_position_target_local_ned()
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE or MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE
        msg.vx = xVel
        msg.vy = yVel
        msg.vz = zVel
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, listener)
    }

    fun sendGuidedPositionAndVelocity(drone: MavLinkDrone?, latitude: Double, longitude: Double, altitude: Double,
                                      xVel: Double, yVel: Double, zVel: Double) {
        drone ?: return
        
        val msg = msg_set_position_target_global_int()
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.toShort()
        msg.lat_int = (latitude * 1E7).toInt()
        msg.lon_int = (longitude * 1E7).toInt()
        msg.alt = altitude.toFloat()
        msg.vx = xVel.toFloat()
        msg.vy = yVel.toFloat()
        msg.vz = zVel.toFloat()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        drone.mavClient?.sendMessage(msg, null)
    }

    fun changeFlightMode(drone: MavLinkDrone?, mode: ApmModes, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_set_mode()
        msg.target_system = drone.sysid
        msg.base_mode = 1 // TODO use meaningful constant
        msg.custom_mode = mode.number
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun setConditionYaw(drone: MavLinkDrone?, targetAngle: Float, yawRate: Float, isClockwise: Boolean,
                        isRelative: Boolean, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_CONDITION_YAW
        msg.param1 = targetAngle
        msg.param2 = yawRate
        msg.param3 = if (isClockwise) 1.toFloat() else -1.toFloat()
        msg.param4 = if (isRelative) 1.toFloat() else 0.toFloat()
        drone.mavClient?.sendMessage(msg, listener)
    }

    /**
     * API for sending manually control to the vehicle using standard joystick axes nomenclature, along with a joystick-like input device.
     * Unused axes can be disabled and buttons are also transmit as boolean values.
     *
     * @param drone
     * @param x        X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
     * @param y        Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
     * @param z        Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
     * @param r        R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
     * @param buttons  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
     * @param listener
     * @see [https://pixhawk.ethz.ch/mavlink/.MANUAL_CONTROL](MANUAL_CONTROL)
     */
    @JvmStatic
    fun sendManualControl(drone: MavLinkDrone?, x: Short, y: Short, z: Short, r: Short, buttons: Int, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_manual_control()
        msg.target = drone.sysid
        msg.x = x
        msg.y = y
        msg.z = z
        msg.r = r
        msg.buttons = buttons
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun sendTakeoff(drone: MavLinkDrone?, alt: Double, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_NAV_TAKEOFF
        msg.param7 = alt.toFloat()
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun sendNavLand(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_NAV_LAND
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun sendNavRTL(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun sendPause(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_OVERRIDE_GOTO
        msg.param1 = MAV_GOTO.MAV_GOTO_DO_HOLD.toFloat()
        msg.param2 = MAV_GOTO.MAV_GOTO_HOLD_AT_CURRENT_POSITION.toFloat()
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun startMission(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_MISSION_START
        drone.mavClient?.sendMessage(msg, listener)
    }

    @JvmStatic
    fun sendArmMessage(drone: MavLinkDrone?, arm: Boolean, emergencyDisarm: Boolean, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM
        msg.param1 = if (arm) 1.toFloat() else 0.toFloat()
        msg.param2 = if (emergencyDisarm) EMERGENCY_DISARM_MAGIC_NUMBER.toFloat() else 0.toFloat()
        msg.param3 = 0f
        msg.param4 = 0f
        msg.param5 = 0f
        msg.param6 = 0f
        msg.param7 = 0f
        msg.confirmation = 0
        Log.v(TAG, "send arm message")
        drone.mavClient?.sendMessage(msg, listener)
        Log.v(TAG, "sent arm message")
    }

    @JvmStatic
    fun sendFlightTermination(drone: MavLinkDrone?, listener: ICommandListener?) {
        drone ?: return
        
        val msg = msg_command_long()
        msg.target_system = drone.sysid
        msg.target_component = drone.compid
        msg.command = MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION
        msg.param1 = 1f
        drone.mavClient?.sendMessage(msg, listener)
    }
}
