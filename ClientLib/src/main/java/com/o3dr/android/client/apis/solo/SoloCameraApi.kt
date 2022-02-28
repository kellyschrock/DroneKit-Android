package com.o3dr.android.client.apis.solo

import android.os.Bundle
import android.view.Surface
import com.MAVLink.enums.GOPRO_COMMAND
import com.o3dr.android.client.Drone
import com.o3dr.android.client.apis.Api.Builder
import com.o3dr.android.client.apis.CameraApi
import com.o3dr.android.client.apis.CapabilityApi
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproConstants
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproConstants.CaptureMode
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproConstants.RecordCommand
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproRecord
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproSetExtendedRequest
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproSetRequest
import com.o3dr.services.android.lib.model.AbstractCommandListener
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to the solo video specific functionality
 * Created by Fredia Huya-Kouadio on 7/12/15.
 *
 * @since 2.5.0
 */
class SoloCameraApi private constructor(drone: Drone) : SoloApi(drone) {
    private val capabilityChecker: CapabilityApi = CapabilityApi.getApi(drone)
    private val cameraApi: CameraApi  = CameraApi.getApi(drone)

    /**
     * Take a photo with the connected gopro.
     *
     * @param listener Register a callback to receive update of the command execution status.
     * @since 2.5.0
     */
    fun takePhoto(listener: AbstractCommandListener?) {
        //Set the gopro to photo mode
        switchCameraCaptureMode(SoloGoproConstants.CAPTURE_MODE_PHOTO, object : AbstractCommandListener() {
            override fun onSuccess() {
                //Send the command to take a picture.
                val photoRecord = SoloGoproRecord(SoloGoproConstants.START_RECORDING)
                sendMessage(photoRecord, listener)
            }

            override fun onError(executionError: Int) {
                listener?.onError(executionError)
            }

            override fun onTimeout() {
                listener?.onTimeout()
            }
        })
    }

    /**
     * Toggle video recording on the connected gopro.
     *
     * @param listener Register a callback to receive update of the command execution status.
     * @since 2.5.0
     */
    fun toggleVideoRecording(listener: AbstractCommandListener?) {
        sendVideoRecordingCommand(SoloGoproConstants.TOGGLE_RECORDING, listener)
    }

    /**
     * Starts video recording on the connected gopro.
     *
     * @param listener Register a callback to receive update of the command execution status.
     * @since 2.5.0
     */
    fun startVideoRecording(listener: AbstractCommandListener?) {
        sendVideoRecordingCommand(SoloGoproConstants.START_RECORDING, listener)
    }

    /**
     * Stops video recording on the connected gopro.
     *
     * @param listener Register a callback to receive update of the command execution status.
     * @since 2.5.0
     */
    fun stopVideoRecording(listener: AbstractCommandListener?) {
        sendVideoRecordingCommand(SoloGoproConstants.STOP_RECORDING, listener)
    }

    private fun sendVideoRecordingCommand(@RecordCommand recordCommand: Int,
                                          listener: AbstractCommandListener?) {
        //Set the gopro to video mode
        switchCameraCaptureMode(SoloGoproConstants.CAPTURE_MODE_VIDEO, object : AbstractCommandListener() {
            override fun onSuccess() {
                //Send the command to toggle video recording
                val videoToggle = SoloGoproRecord(recordCommand)
                sendMessage(videoToggle, listener)
            }

            override fun onError(executionError: Int) {
                listener?.onError(executionError)
            }

            override fun onTimeout() {
                listener?.onTimeout()
            }
        })
    }

    fun startVideoStream(surface: Surface?, tag: String?,
                         listener: AbstractCommandListener?) {
        startVideoStream(surface, tag, false, listener)
    }

    /**
     * Attempt to grab ownership and start the video stream from the connected drone. Can fail if
     * the video stream is already owned by another client.
     *
     * @param surface  Surface object onto which the video is decoded.
     * @param tag      Video tag.
     * @param enableLocalRecording  Set to true to enable local recording, false to disable it.
     * @param listener Register a callback to receive update of the command execution status.
     *
     * @since 2.5.0
     */
    fun startVideoStream(surface: Surface?, tag: String?, enableLocalRecording: Boolean,
                         listener: AbstractCommandListener?) {
        if (surface == null) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }
        capabilityChecker.checkFeatureSupport(CapabilityApi.FeatureIds.SOLO_VIDEO_STREAMING
        ) { featureId, result, resultInfo ->
            when (result) {
                CapabilityApi.FEATURE_SUPPORTED -> {
                    val videoProps = Bundle()
                    videoProps.putInt(CameraApi.VIDEO_PROPS_UDP_PORT, SOLO_STREAM_UDP_PORT)
                    videoProps.putBoolean(CameraApi.VIDEO_ENABLE_LOCAL_RECORDING, enableLocalRecording)
                    if (enableLocalRecording) {
                        val localRecordingFilename = "solo_stream_" + FILE_DATE_FORMAT.format(Date())
                        videoProps.putString(CameraApi.VIDEO_LOCAL_RECORDING_FILENAME, localRecordingFilename)
                    }
                    cameraApi.startVideoStream(surface, tag, videoProps, listener)
                }
                CapabilityApi.FEATURE_UNSUPPORTED -> postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                else -> postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
        }
    }

    /**
     * Attempt to grab ownership and start the video stream from the connected drone. Can fail if
     * the video stream is already owned by another client.
     *
     * @param surface  Surface object onto which the video is decoded.
     * @param listener Register a callback to receive update of the command execution status.
     *
     * @since 2.5.0
     */
    fun startVideoStream(surface: Surface?, listener: AbstractCommandListener?) {
        startVideoStream(surface, "", listener)
    }

    /**
     * Stop the video stream from the connected drone, and release ownership.
     *
     * @param listener Register a callback to receive update of the command execution status.
     *
     * @since 2.5.0
     */
    fun stopVideoStream(listener: AbstractCommandListener?) {
        stopVideoStream("", listener)
    }

    /**
     * Stop the video stream from the connected drone, and release ownership.
     *
     * @param tag      Video tag.
     * @param listener Register a callback to receive update of the command execution status.
     *
     * @since 2.5.0
     */
    fun stopVideoStream(tag: String?, listener: AbstractCommandListener?) {
        capabilityChecker.checkFeatureSupport(CapabilityApi.FeatureIds.SOLO_VIDEO_STREAMING
        ) { featureId, result, resultInfo ->
            when (result) {
                CapabilityApi.FEATURE_SUPPORTED -> cameraApi.stopVideoStream(tag, listener)
                CapabilityApi.FEATURE_UNSUPPORTED -> postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                else -> postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
        }
    }

    /**
     * Switches the camera capture mode.
     *
     * @param captureMode One of [SoloGoproConstants.CAPTURE_MODE_VIDEO],
     * [SoloGoproConstants.CAPTURE_MODE_PHOTO],
     * [SoloGoproConstants.CAPTURE_MODE_BURST],
     * [SoloGoproConstants.CAPTURE_MODE_TIME_LAPSE]
     * @param listener    Register a callback to receive update of the command execution status.
     * @since 2.6.8
     */
    fun switchCameraCaptureMode(@CaptureMode captureMode: Byte,
                                listener: AbstractCommandListener?) {
        val captureModeRequest = SoloGoproSetRequest(GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE.toShort(), captureMode.toShort())
        sendMessage(captureModeRequest, listener)
    }

    private fun sendExtendedRequest(listener: AbstractCommandListener, command: Int, value1: Byte,
                                    value2: Byte, value3: Byte, value4: Byte) {
        val values = byteArrayOf(value1, value2, value3, value4)
        val extendedRequest = SoloGoproSetExtendedRequest(command.toShort(), values)
        sendMessage(extendedRequest, listener)
    }

    private fun sendExtendedRequest(listener: AbstractCommandListener, command: Int, value: Byte) {
        sendExtendedRequest(listener, command, value, 0.toByte(), 0.toByte(), 0.toByte())
    }

    /**
     * Updates the camera video settings.
     * @since 2.7.0
     * @param resolution
     * @param frameRate
     * @param fieldOfView
     * @param flags
     */
    fun updateVideoSettings(resolution: Byte, frameRate: Byte, fieldOfView: Byte, flags: Byte,
                            listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS, resolution, frameRate,
                fieldOfView, flags)
    }

    fun setCameraPhotoResolution(photoResolution: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION, photoResolution)
    }

    fun enableCameraLowLight(enable: Boolean, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT, if (enable) 1.toByte() else 0.toByte())
    }

    fun setCameraPhotoBurstRate(burstRate: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE, burstRate)
    }

    fun enableCameraProtune(enable: Boolean, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE, if (enable) 1.toByte() else 0.toByte())
    }

    fun setCameraProtuneWhiteBalance(whiteBalance: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE, whiteBalance)
    }

    fun setCameraProtuneColour(colour: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_COLOUR, colour)
    }

    fun setCameraProtuneGain(gain: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN, gain)
    }

    fun setCameraProtuneSharpness(sharpness: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_SHARPNESS, sharpness)
    }

    fun setCameraProtuneExposure(exposure: Byte, listener: AbstractCommandListener) {
        sendExtendedRequest(listener, GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_EXPOSURE, exposure)
    }

    companion object {
        private val FILE_DATE_FORMAT = SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", Locale.US)
        private val soloCameraApiCache = ConcurrentHashMap<Drone, SoloCameraApi>()
        private val apiBuilder: Builder<SoloCameraApi> = Builder { drone -> SoloCameraApi(drone) }
        private const val SOLO_STREAM_UDP_PORT = 5600

        /**
         * Retrieves a sololink api instance.
         *
         * @param drone target vehicle
         * @return a SoloCameraApi instance.
         */
        @JvmStatic
        fun getApi(drone: Drone?): SoloCameraApi {
            return getApi(drone, soloCameraApiCache, apiBuilder)
        }
    }
}
