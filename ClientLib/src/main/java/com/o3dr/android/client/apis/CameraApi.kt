package com.o3dr.android.client.apis

import android.os.Bundle
import android.view.Surface
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.action.CameraActions
import com.o3dr.android.client.apis.CameraApi
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides support to control generic camera functionality
 * Created by Fredia Huya-Kouadio on 10/11/15.
 *
 * @since 2.6.8
 */
class CameraApi private constructor(private val drone: Drone) : Api() {
    /**
     * Attempt to grab ownership and start the video stream from the connected drone. Can fail if
     * the video stream is already owned by another client.
     *
     * @param surface       Surface object onto which the video is decoded.
     * @param tag           Video tag.
     * @param videoProps    Non-null video properties. @see VIDEO_PROPS_UDP_PORT
     * @param listener      Register a callback to receive update of the command execution status.
     * @since 2.6.8
     */
    fun startVideoStream(surface: Surface, tag: String?,
                         videoProps: Bundle, listener: AbstractCommandListener?) {
        if (surface == null || videoProps == null) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }
        val params = Bundle()
        params.putParcelable(CameraActions.EXTRA_VIDEO_DISPLAY, surface)
        params.putString(CameraActions.EXTRA_VIDEO_TAG, tag)
        params.putBundle(CameraActions.EXTRA_VIDEO_PROPERTIES, videoProps)
        drone.performAsyncActionOnDroneThread(Action(CameraActions.ACTION_START_VIDEO_STREAM, params), listener)
    }

    /**
     * Stop the video stream from the connected drone, and release ownership.
     *
     * @param tag      Video tag.
     * @param listener Register a callback to receive update of the command execution status.
     * @since 2.6.8
     */
    fun stopVideoStream(tag: String?, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putString(CameraActions.EXTRA_VIDEO_TAG, tag)
        drone.performAsyncActionOnDroneThread(Action(CameraActions.ACTION_STOP_VIDEO_STREAM, params), listener)
    }

    companion object {
        private val apiCache = ConcurrentHashMap<Drone, CameraApi>()
        private val apiBuilder: Builder<CameraApi> = Builder { drone -> CameraApi(drone) }

        /**
         * Used to specify the udp port from which to access the streamed video.
         */
        const val VIDEO_PROPS_UDP_PORT = CameraActions.EXTRA_VIDEO_PROPS_UDP_PORT

        /**
         * Used to specify which IP from which to access the streamed video.
         */
        const val VIDEO_PROPS_UDP_IP = CameraActions.EXTRA_VIDEO_PROPS_UDP_IP

        /**
         * Key to specify whether to enable/disable local recording of the video stream.
         * @since 2.7.0
         */
        const val VIDEO_ENABLE_LOCAL_RECORDING = CameraActions.EXTRA_VIDEO_ENABLE_LOCAL_RECORDING

        /**
         * Key to specify the filename to use for the local recording.
         * @since 2.7.0
         */
        const val VIDEO_LOCAL_RECORDING_FILENAME = CameraActions.EXTRA_VIDEO_LOCAL_RECORDING_FILENAME

        /**
         * Retrieves a camera api instance
         *
         * @param drone
         * @return
         */
        @JvmStatic
        fun getApi(drone: Drone?): CameraApi {
            return getApi(drone, apiCache, apiBuilder)
        }
    }
}
