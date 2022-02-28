package com.o3dr.android.client.apis

import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import timber.log.Timber
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.drone.action.ConnectionActions
import kotlin.jvm.JvmOverloads
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.drone.action.StateActions
import com.o3dr.services.android.lib.drone.property.VehicleMode
import com.o3dr.services.android.lib.drone.action.ParameterActions
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.android.client.apis.VehicleApi
import com.o3dr.services.android.lib.drone.property.Parameters
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to the vehicle specific functionality.
 */
class VehicleApi private constructor(private val drone: Drone) : Api() {
    /**
     * Establish connection with the vehicle.
     *
     * @param parameter parameter for the connection.
     */
    fun connect(parameter: ConnectionParameter?) {
        Timber.d("connect(): param=%s", parameter)
        val params = Bundle()
        params.putParcelable(ConnectionActions.EXTRA_CONNECT_PARAMETER, parameter)
        val connectAction = Action(ConnectionActions.ACTION_CONNECT, params)
        Timber.d("performAsyncAction on %s", drone)
        drone.performAsyncAction(connectAction)
    }

    /**
     * Break connection with the vehicle.
     */
    fun disconnect() {
        drone.performAsyncAction(Action(ConnectionActions.ACTION_DISCONNECT))
    }
    /**
     * Arm or disarm the connected drone.
     *
     * @param arm      true to arm, false to disarm.
     * @param listener Register a callback to receive update of the command execution state.
     */
    /**
     * Arm or disarm the connected drone.
     *
     * @param arm true to arm, false to disarm.
     */
    @JvmOverloads
    fun arm(arm: Boolean, listener: AbstractCommandListener? = null) {
        arm(arm, false, listener)
    }

    /**
     * Arm or disarm the connected drone.
     *
     * @param arm             true to arm, false to disarm.
     * @param emergencyDisarm true to skip landing check and disarm immediately,
     * false to disarm only if it is safe to do so.
     * @param listener        Register a callback to receive update of the command execution state.
     */
    fun arm(arm: Boolean, emergencyDisarm: Boolean, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putBoolean(StateActions.EXTRA_ARM, arm)
        params.putBoolean(StateActions.EXTRA_EMERGENCY_DISARM, emergencyDisarm)
        drone.performAsyncActionOnDroneThread(Action(StateActions.ACTION_ARM, params), listener)
    }

    /**
     * Change the vehicle mode for the connected drone.
     *
     * @param newMode new vehicle mode.
     */
    fun setVehicleMode(newMode: VehicleMode?) {
        setVehicleMode(newMode, null)
    }

    /**
     * Change the vehicle mode for the connected drone.
     *
     * @param newMode  new vehicle mode.
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun setVehicleMode(newMode: VehicleMode?, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putParcelable(StateActions.EXTRA_VEHICLE_MODE, newMode)
        drone.performAsyncActionOnDroneThread(Action(StateActions.ACTION_SET_VEHICLE_MODE, params), listener)
    }

    /**
     * Generate action used to refresh the parameters for the connected drone.
     */
    fun refreshParameters() {
        drone.performAsyncAction(Action(ParameterActions.ACTION_REFRESH_PARAMETERS))
    }

    /**
     * Generate action used to write the given parameters to the connected drone.
     *
     * @param parameters parameters to write to the drone.
     * @return
     */
    fun writeParameters(parameters: Parameters?) {
        val params = Bundle()
        params.putParcelable(ParameterActions.EXTRA_PARAMETERS, parameters)
        drone.performAsyncAction(Action(ParameterActions.ACTION_WRITE_PARAMETERS, params))
    }

    /**
     * Changes the vehicle home location.
     *
     * @param homeLocation New home coordinate
     * @param listener     Register a callback to receive update of the command execution state.
     */
    fun setVehicleHome(homeLocation: LatLongAlt?, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putParcelable(StateActions.EXTRA_VEHICLE_HOME_LOCATION, homeLocation)
        drone.performAsyncActionOnDroneThread(Action(StateActions.ACTION_SET_VEHICLE_HOME, params), listener)
    }

    /**
     * Enables 'return to me'
     * @param isEnabled
     * @param listener
     */
    fun enableReturnToMe(isEnabled: Boolean, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putBoolean(StateActions.EXTRA_IS_RETURN_TO_ME_ENABLED, isEnabled)
        drone.performAsyncActionOnDroneThread(Action(StateActions.ACTION_ENABLE_RETURN_TO_ME, params), listener)
    }

    /**
     * Update the vehicle data stream rate.
     *
     * Note: This is ineffective for Solo vehicles since their data stream rate is handled
     * by the onboard companion computer.
     *
     * @param rate          The new data stream rate
     * @param listener      Register a callback to receive update of the command execution state
     * @since 2.9.0
     */
    fun updateVehicleDataStreamRate(rate: Int, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putInt(StateActions.EXTRA_VEHICLE_DATA_STREAM_RATE, rate)
        drone.performAsyncActionOnDroneThread(Action(StateActions.ACTION_UPDATE_VEHICLE_DATA_STREAM_RATE, params), listener)
    }

    companion object {
        private val vehicleApiCache = ConcurrentHashMap<Drone, VehicleApi>()
        private val apiBuilder: Builder<VehicleApi> = Builder { drone -> VehicleApi(drone) }

        /**
         * Retrieves a vehicle api instance.
         *
         * @param drone target vehicle
         * @return a VehicleApi instance.
         */
        @JvmStatic
        fun getApi(drone: Drone?): VehicleApi {
            return getApi(drone, vehicleApiCache, apiBuilder)
        }
    }
}
