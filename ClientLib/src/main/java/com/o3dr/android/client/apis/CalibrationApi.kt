package com.o3dr.android.client.apis

import kotlin.jvm.JvmOverloads
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.gcs.action.CalibrationActions
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.android.client.apis.CalibrationApi
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to the calibration specific functionality.
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
class CalibrationApi private constructor(private val drone: Drone) : Api() {
    /**
     * Start the imu calibration.
     *
     * @param listener Register a callback to receive update of the command execution state.
     */
    /**
     * Start the imu calibration.
     */
    @JvmOverloads
    fun startIMUCalibration(listener: AbstractCommandListener? = null) {
        drone.performAsyncActionOnDroneThread(Action(CalibrationActions.ACTION_START_IMU_CALIBRATION), listener)
    }

    /**
     * Generate an action to send an imu calibration acknowledgement.
     */
    fun sendIMUAck(step: Int) {
        val params = Bundle()
        params.putInt(CalibrationActions.EXTRA_IMU_STEP, step)
        drone.performAsyncAction(Action(CalibrationActions.ACTION_SEND_IMU_CALIBRATION_ACK, params))
    }
    /**
     * Start the magnetometer calibration process
     *
     * @param retryOnFailure    if true, automatically retry the magnetometer calibration if it fails
     * @param saveAutomatically if true, save the calibration automatically without user input.
     * @param startDelay        positive delay in seconds before starting the calibration
     */
    /**
     * Start the magnetometer calibration process.
     */
    @JvmOverloads
    fun startMagnetometerCalibration(retryOnFailure: Boolean = false, saveAutomatically: Boolean = true, startDelay: Int = 0) {
        val params = Bundle()
        params.putBoolean(CalibrationActions.EXTRA_RETRY_ON_FAILURE, retryOnFailure)
        params.putBoolean(CalibrationActions.EXTRA_SAVE_AUTOMATICALLY, saveAutomatically)
        params.putInt(CalibrationActions.EXTRA_START_DELAY, startDelay)
        drone.performAsyncAction(Action(CalibrationActions.ACTION_START_MAGNETOMETER_CALIBRATION, params))
    }

    /**
     * Confirm the result of the magnetometer calibration.
     */
    fun acceptMagnetometerCalibration() {
        drone.performAsyncAction(Action(CalibrationActions.ACTION_ACCEPT_MAGNETOMETER_CALIBRATION))
    }

    /**
     * Cancel the magnetometer calibration is one if running.
     */
    fun cancelMagnetometerCalibration() {
        drone.performAsyncAction(Action(CalibrationActions.ACTION_CANCEL_MAGNETOMETER_CALIBRATION))
    }

    companion object {
        private val calibrationApiCache = ConcurrentHashMap<Drone, CalibrationApi>()
        private val apiBuilder: Builder<CalibrationApi> = Builder { drone -> CalibrationApi(drone) }

        /**
         * Retrieves a CalibrationApi instance.
         *
         * @param drone target vehicle.
         * @return a CalibrationApi instance.
         */
        @JvmStatic
        fun getApi(drone: Drone?): CalibrationApi {
            return getApi(drone, calibrationApiCache, apiBuilder)
        }
    }
}
