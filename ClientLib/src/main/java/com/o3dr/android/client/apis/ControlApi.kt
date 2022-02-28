package com.o3dr.android.client.apis

import com.o3dr.services.android.lib.model.AbstractCommandListener
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.drone.action.ControlActions
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import timber.log.Timber
import com.o3dr.android.client.apis.ControlApi
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.android.client.apis.ControlApi.ManualControlStateListener
import org.droidplanner.services.android.impl.msg.msg_do_change_speed
import com.o3dr.services.android.lib.drone.action.ExperimentalActions
import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition.TargetState
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to the vehicle control functionality.
 *
 *
 * Use of this api might required the vehicle to be in a specific flight mode (i.e: GUIDED)
 *
 *
 * Created by Fredia Huya-Kouadio on 9/7/15.
 */
class ControlApi private constructor(private val drone: Drone) : Api() {
    /**
     * Perform a guided take off.
     *
     * @param altitude altitude in meters
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun takeoff(altitude: Double, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putDouble(ControlActions.EXTRA_ALTITUDE, altitude)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_DO_GUIDED_TAKEOFF, params), listener)
    }

    /**
     * Pause the vehicle at its current location.
     *
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun pauseAtCurrentLocation(listener: AbstractCommandListener?) {
        val params = Bundle()
        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_SEND_BRAKE_VEHICLE, params), listener)
    }

    /**
     * Instructs the vehicle to go to the specified location.
     *
     * @param point    target location
     * @param force    true to enable guided mode is required.
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun goTo(point: LatLongAlt?, force: Boolean, listener: AbstractCommandListener?) {
        Timber.d("goTo(): point=%s force=%s", point, force)
        val params = Bundle().apply {
            putBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT, force)
            putParcelable(ControlActions.EXTRA_GUIDED_POINT, point)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_SEND_GUIDED_POINT, params), listener)
    }

    /**
     * Instructs the vehicle to go to the specified location. Hopefully.
     *
     * @param point    target location
     */
    fun sendGuidedPointDirect(point: LatLongAlt?) {
        Timber.d("sendGuidedPointDirect(): point=%s", point)
        val params = Bundle().apply {
            putParcelable(ControlActions.EXTRA_GUIDED_POINT, point)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_SEND_GUIDED_POINT_DIRECT, params), null)
    }

    /**
     * Instructs the vehicle to orient toward the specified location
     *
     * @param point
     * @param force
     * @param listener
     * @since 2.9.0
     */
    fun lookAt(point: LatLongAlt?, force: Boolean, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT, force)
            putParcelable(ControlActions.EXTRA_LOOK_AT_TARGET, point)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_LOOK_AT_TARGET, params), listener)
    }

    fun resetROI(listener: AbstractCommandListener?) {
        val params = Bundle()
        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_RESET_ROI, params), listener)
    }

    /**
     * Instructs the vehicle to climb to the specified altitude.
     *
     * @param altitude altitude in meters
     */
    fun climbTo(altitude: Double) {
        Timber.d("climbTo(): altitude=%.1f", altitude)
        val params = Bundle().apply {
            putDouble(ControlActions.EXTRA_ALTITUDE, altitude)
        }

        drone.performAsyncAction(Action(ControlActions.ACTION_SET_GUIDED_ALTITUDE, params))
    }

    /**
     * Instructs the vehicle to turn to the specified target angle
     *
     * @param targetAngle Target angle in degrees [0-360], with 0 == north.
     * @param turnRate    Turning rate normalized to the range [-1.0f, 1.0f]. Positive values for clockwise turns, and negative values for counter-clockwise turns.
     * @param isRelative  True is the target angle is relative to the current vehicle attitude, false otherwise if it's absolute.
     * @param listener    Register a callback to receive update of the command execution state.
     */
    fun turnTo(targetAngle: Float, turnRate: Float, isRelative: Boolean, listener: AbstractCommandListener?) {
        if (!isWithinBounds(targetAngle, 0f, 360f) || !isWithinBounds(turnRate, -1.0f, 1.0f)) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }

        val params = Bundle().apply {
            putFloat(ControlActions.EXTRA_YAW_TARGET_ANGLE, targetAngle)
            putFloat(ControlActions.EXTRA_YAW_CHANGE_RATE, turnRate)
            putBoolean(ControlActions.EXTRA_YAW_IS_RELATIVE, isRelative)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_SET_CONDITION_YAW, params), listener)
    }

    /**
     * Move the vehicle along the specified normalized velocity vector.
     *
     * @param vx       x velocity normalized to the range [-1.0f, 1.0f]. Generally correspond to the pitch of the vehicle.
     * @param vy       y velocity normalized to the range [-1.0f, 1.0f]. Generally correspond to the roll of the vehicle.
     * @param vz       z velocity normalized to the range [-1.0f, 1.0f]. Generally correspond to the thrust of the vehicle.
     * @param listener Register a callback to receive update of the command execution state.
     * @since 2.6.9
     */
    fun manualControl(vx: Float, vy: Float, vz: Float, listener: AbstractCommandListener?) {
        if (!isWithinBounds(vx, -1f, 1f) || !isWithinBounds(vy, -1f, 1f) || !isWithinBounds(vz, -1f, 1f)) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }

        val params = Bundle().apply {
            putFloat(ControlActions.EXTRA_VELOCITY_X, vx)
            putFloat(ControlActions.EXTRA_VELOCITY_Y, vy)
            putFloat(ControlActions.EXTRA_VELOCITY_Z, vz)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_SET_VELOCITY, params), listener)
    }

    /**
     * [Dis|En]able manual control on the vehicle.
     * The result of the action will be conveyed through the passed listener.
     *
     * @param enable   True to enable manual control, false to disable.
     * @param listener Register a callback to receive the result of the operation.
     * @since 2.6.9
     */
    fun enableManualControl(enable: Boolean, listener: ManualControlStateListener?) {
        val listenerWrapper: AbstractCommandListener? = if (listener == null) null else object : AbstractCommandListener() {
            override fun onSuccess() {
                if (enable) {
                    listener.onManualControlToggled(true)
                } else {
                    listener.onManualControlToggled(false)
                }
            }

            override fun onError(executionError: Int) {
                if (enable) {
                    listener.onManualControlToggled(false)
                }
            }

            override fun onTimeout() {
                if (enable) {
                    listener.onManualControlToggled(false)
                }
            }
        }

        val params = Bundle().apply {
            putBoolean(ControlActions.EXTRA_DO_ENABLE, enable)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_ENABLE_MANUAL_CONTROL, params), listenerWrapper)
    }

    fun changeGuidedSpeed(speedType: Int, targetMetersSecond: Int, throttlePercent: Int) {
        val msg = msg_do_change_speed(
                speedType.toShort(), targetMetersSecond.toShort(), throttlePercent.toShort())
        val params = Bundle().apply {
            putParcelable(ExperimentalActions.EXTRA_MAVLINK_MESSAGE, MavlinkMessageWrapper(msg))
        }

        drone.performAsyncAction(Action(ExperimentalActions.ACTION_SEND_MAVLINK_MESSAGE, params))
    }

    /**
     * Instructs the vehicle to perform a VTOL state transition.
     *
     * @param state
     */
    fun setVTOLState(state: TargetState) {
        Timber.d("setVTOLState(%s)", state)
        val params = Bundle().apply {
            putInt(ControlActions.EXTRA_VTOL_TARGET_STATE, state.ordinal)
        }

        drone.performAsyncActionOnDroneThread(Action(ControlActions.ACTION_VTOL_TRANSITION, params), null)
    }

    /**
     * Used to monitor the state of manual control for the vehicle.
     *
     * @since 2.6.9
     */
    interface ManualControlStateListener {
        /**
         * Manual control is toggled on the vehicle.
         * @param isEnabled True if manual control is enabled, false if disabled.
         */
        fun onManualControlToggled(isEnabled: Boolean)
    }

    companion object {
        private val apiCache = ConcurrentHashMap<Drone, ControlApi>()
        private val apiBuilder: Builder<ControlApi> = Builder { drone -> ControlApi(drone) }

        /**
         * Retrieves a control api instance.
         *
         * @param drone
         * @return
         */
        @JvmStatic
        fun getApi(drone: Drone?): ControlApi {
            return getApi(drone, apiCache, apiBuilder)
        }

        private fun isWithinBounds(value: Float, lowerBound: Float, upperBound: Float): Boolean {
            return value in lowerBound..upperBound
        }
    }
}
