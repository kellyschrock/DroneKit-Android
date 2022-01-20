package com.o3dr.android.client.apis

import com.o3dr.android.client.interfaces.DroneListener
import com.o3dr.android.client.apis.GimbalApi.GimbalOrientation
import com.o3dr.android.client.apis.GimbalApi.GimbalOrientationListener
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import android.os.Bundle
import com.MAVLink.enums.MAV_MOUNT_MODE
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.drone.action.GimbalActions
import com.o3dr.services.android.lib.model.SimpleCommandListener
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra
import com.o3dr.android.client.apis.GimbalApi
import com.o3dr.services.android.lib.drone.property.Type
import com.o3dr.services.android.lib.model.action.Action
import org.jetbrains.annotations.Nullable
import java.lang.NullPointerException
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue

class GimbalApi private constructor(private val drone: Drone) : Api(), DroneListener {
    interface GimbalOrientationListener {
        /**
         * Called when the gimbal orientation is updated.
         * @param orientation GimbalOrientation object
         */
        fun onGimbalOrientationUpdate(orientation: GimbalOrientation?)

        /**
         * Indicates errors occurring from attempting to set the gimbal orientation.
         * @param error @see [com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError]
         */
        fun onGimbalOrientationCommandError(error: Int)
    }

    /**
     * Stores the gimbal orientation angles.
     */
    class GimbalOrientation {
        var pitch = 0f
            private set
        var roll = 0f
            private set
        var yaw = 0f
            private set

        fun updateOrientation(pitch: Float, roll: Float, yaw: Float) {
            this.pitch = pitch
            this.roll = roll
            this.yaw = yaw
        }

        constructor() {}

        constructor(source: GimbalOrientation) {
            pitch = source.pitch
            roll = source.roll
            yaw = source.yaw
        }

        override fun equals(o: Any?): Boolean {
            if (this === o) return true
            if (o !is GimbalOrientation) return false
            val that = o
            if (java.lang.Float.compare(that.pitch, pitch) != 0) return false
            return if (java.lang.Float.compare(that.roll, roll) != 0) false else java.lang.Float.compare(that.yaw, yaw) == 0
        }

        override fun hashCode(): Int {
            var result = if (pitch != +0.0f) java.lang.Float.floatToIntBits(pitch) else 0
            result = 31 * result + if (roll != +0.0f) java.lang.Float.floatToIntBits(roll) else 0
            result = 31 * result + if (yaw != +0.0f) java.lang.Float.floatToIntBits(yaw) else 0
            return result
        }

        override fun toString(): String {
            return "GimbalOrientation{" +
                    "pitch=" + pitch +
                    ", roll=" + roll +
                    ", yaw=" + yaw +
                    '}'
        }
    }

    private val gimbalListeners = ConcurrentLinkedQueue<GimbalOrientationListener>()
    private val gimbalOrientation = GimbalOrientation()
    fun getGimbalOrientation(): GimbalOrientation {
        return GimbalOrientation(gimbalOrientation)
    }

    /**
     * Enables control of the gimbal. After calling this method, use [GimbalApi.updateGimbalOrientation]
     * to update the gimbal orientation.
     * @param listener non-null GimbalStatusListener callback.
     */
    fun startGimbalControl(listener: GimbalOrientationListener?) {
        if (listener == null) throw NullPointerException("Listener can't be null.")
        val vehicleType = drone.getAttribute<Type>(AttributeType.TYPE)
        if (vehicleType!!.droneType != Type.TYPE_COPTER) {
            drone.post { listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_UNSUPPORTED) }
            return
        }
        gimbalListeners.add(listener)
        configureGimbalMountMode(listener)
    }

    private fun configureGimbalMountMode(listener: GimbalOrientationListener) {
        val params = Bundle(1)
        params.putInt(GimbalActions.GIMBAL_MOUNT_MODE, MAV_MOUNT_MODE.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        drone.performAsyncActionOnDroneThread(Action(GimbalActions.ACTION_SET_GIMBAL_MOUNT_MODE, params), object : SimpleCommandListener() {
            override fun onTimeout() {
                listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_FAILED)
            }

            override fun onError(error: Int) {
                listener.onGimbalOrientationCommandError(error)
            }
        })
    }

    /**
     * Disables control of the gimbal. After calling this method, no call to [GimbalApi.updateGimbalOrientation]
     * will be allowed.
     * @param listener non-null GimbalStatusListener callback.
     *
     * @since 2.5.0
     */
    fun stopGimbalControl(listener: GimbalOrientationListener?) {
        if (listener == null) throw NullPointerException("Listener can't be null.")
        if (!gimbalListeners.contains(listener)) {
            drone.post { listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_DENIED) }
            return
        }
        gimbalListeners.remove(listener)

        //Reset the gimbal mount to the default.
        val params = Bundle(1)
        params.putInt(GimbalActions.GIMBAL_MOUNT_MODE, MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING)
        drone.performAsyncActionOnDroneThread(Action(GimbalActions.ACTION_SET_GIMBAL_MOUNT_MODE, params), object : SimpleCommandListener() {
            override fun onTimeout() {
                listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_FAILED)
            }

            override fun onError(error: Int) {
                listener.onGimbalOrientationCommandError(error)
            }
        })
    }

    /**
     * Set the orientation of the gimbal
     * @param orientation Desired orientation values.
     * @param listener Register a callback to receive update of the command execution state. Must be non-null.
     * @since 2.8.0
     */
    fun updateGimbalOrientation(orientation: GimbalOrientation, listener: GimbalOrientationListener) {
        updateGimbalOrientation(orientation.pitch, orientation.roll, orientation.yaw, listener)
    }

    /**
     * Set the orientation of a gimbal
     *
     * @param pitch       the desired gimbal pitch in degrees. 0 is straight forwards, -90 is straight down
     * @param roll       the desired gimbal roll in degrees
     * @param yaw       the desired gimbal yaw in degrees
     * @param listener Register a callback to receive update of the command execution state. Must be non-null.
     * @since 2.5.0
     */
    fun updateGimbalOrientation(pitch: Float, roll: Float, yaw: Float, listener: GimbalOrientationListener) {
        if (listener == null) throw NullPointerException("Listener must be non-null.")
        if (!gimbalListeners.contains(listener)) {
            drone.post { listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_DENIED) }
            return
        }
        val params = Bundle()
        params.putFloat(GimbalActions.GIMBAL_PITCH, pitch)
        params.putFloat(GimbalActions.GIMBAL_ROLL, roll)
        params.putFloat(GimbalActions.GIMBAL_YAW, yaw)
        drone.performAsyncActionOnDroneThread(Action(GimbalActions.ACTION_SET_GIMBAL_ORIENTATION, params), object : SimpleCommandListener() {
            override fun onTimeout() {
                listener.onGimbalOrientationCommandError(CommandExecutionError.COMMAND_FAILED)
            }

            override fun onError(error: Int) {
                listener.onGimbalOrientationCommandError(error)
            }
        })
    }

    private fun notifyGimbalOrientationUpdated(orientation: GimbalOrientation) {
        if (gimbalListeners.isEmpty()) return
        for (listener in gimbalListeners) {
            listener.onGimbalOrientationUpdate(orientation)
        }
    }

    override fun onDroneEvent(event: String?, extras: Bundle?) {
        when (event) {
            AttributeEvent.GIMBAL_ORIENTATION_UPDATED -> {
                extras?.let {
                    val pitch = it.getFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_PITCH)
                    val roll = it.getFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_ROLL)
                    val yaw = it.getFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_YAW)
                    gimbalOrientation.updateOrientation(pitch, roll, yaw)
                    notifyGimbalOrientationUpdated(gimbalOrientation)
                }
            }
        }
    }

    override fun onDroneServiceInterrupted(errorMsg: String) {}

    companion object {
        private val gimbalApiCache = ConcurrentHashMap<Drone, GimbalApi>()
        private val apiBuilder: Builder<GimbalApi> = Builder { drone -> GimbalApi(drone) }

        @JvmStatic
        fun getApi(drone: Drone?): GimbalApi {
            return getApi(drone, gimbalApiCache, apiBuilder)
        }
    }

    init {
        drone.registerDroneListener(this)
    }
}
