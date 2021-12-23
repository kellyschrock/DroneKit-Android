package com.o3dr.android.client.apis

import com.o3dr.android.client.apis.CapabilityApi
import com.o3dr.android.client.apis.ExperimentalApi.VideoStreamObserver
import com.o3dr.services.android.lib.drone.action.ExperimentalActions
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.model.AbstractCommandListener
import android.os.Bundle
import android.os.Handler
import android.text.TextUtils
import android.util.Log
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.android.client.apis.ExperimentalApi.IVideoStreamCallback
import com.o3dr.android.client.apis.CapabilityApi.FeatureSupportListener
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.action.CameraActions
import com.o3dr.android.client.utils.connection.IpConnectionListener
import com.o3dr.android.client.apis.ExperimentalApi
import com.o3dr.android.client.utils.connection.UdpConnection
import com.o3dr.services.android.lib.model.action.Action
import java.lang.NullPointerException
import java.nio.ByteBuffer
import java.util.concurrent.ConcurrentHashMap

/**
 * Contains drone commands with no defined interaction model yet.
 */
class ExperimentalApi private constructor(private val drone: Drone) : Api() {
    private val capabilityChecker: CapabilityApi  = CapabilityApi.getApi(drone)
    private val videoStreamObserver: VideoStreamObserver = VideoStreamObserver(drone.handler!!)

    /**
     * Triggers the camera.
     */
    fun triggerCamera() {
        drone.performAsyncAction(Action(ExperimentalActions.ACTION_TRIGGER_CAMERA))
    }

    /**
     * Specify a region of interest for the vehicle to point at.
     *
     * @param roi Region of interest coordinate.
     */
    fun setROI(roi: LatLongAlt?) {
        setROI(roi, null)
    }

    /**
     * Specify a region of interest for the vehicle to point at.
     *
     * @param roi      Region of interest coordinate.
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun setROI(roi: LatLongAlt?, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putParcelable(ExperimentalActions.EXTRA_SET_ROI_LAT_LONG_ALT, roi)
        val epmAction = Action(ExperimentalActions.ACTION_SET_ROI, params)
        drone.performAsyncActionOnDroneThread(epmAction, listener)
    }

    /**
     * This is an advanced/low-level method to send raw mavlink to the vehicle.
     *
     *
     * This method is included as an ‘escape hatch’ to allow developers to make progress if we’ve
     * somehow missed providing some essential operation in the rest of this API. Callers do
     * not need to populate sysId/componentId/crc in the packet, this method will take care of that
     * before sending.
     *
     *
     * If you find yourself needing to use this method please contact the drone-platform google
     * group and we’ll see if we can support the operation you needed in some future revision of
     * the API.
     *
     * @param messageWrapper A MAVLinkMessage wrapper instance. No need to fill in
     * sysId/compId/seqNum - the API will take care of that.
     */
    fun sendMavlinkMessage(messageWrapper: MavlinkMessageWrapper?) {
        val params = Bundle()
        params.putParcelable(ExperimentalActions.EXTRA_MAVLINK_MESSAGE, messageWrapper)
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_SEND_MAVLINK_MESSAGE, params), null)
    }

    fun sendMavlinkMessage(messageWrapper: MavlinkMessageWrapper?, targetSys: Short, targetComponent: Short) {
        val params = Bundle()
        params.putParcelable(ExperimentalActions.EXTRA_MAVLINK_MESSAGE, messageWrapper)
        params.putShort(ExperimentalActions.EXTRA_TARGET_SYS, targetSys)
        params.putShort(ExperimentalActions.EXTRA_TARGET_COMPONENT, targetComponent)
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_SEND_MAVLINK_MESSAGE, params), null)
    }

    /**
     * Set a Relay pin’s voltage high or low
     *
     * @param relayNumber
     * @param enabled     true for relay to be on, false for relay to be off.
     */
    fun setRelay(relayNumber: Int, enabled: Boolean) {
        setRelay(relayNumber, enabled, null)
    }

    /**
     * Set a Relay pin’s voltage high or low
     *
     * @param relayNumber
     * @param enabled     true for relay to be on, false for relay to be off.
     * @param listener    Register a callback to receive update of the command execution state.
     */
    fun setRelay(relayNumber: Int, enabled: Boolean, listener: AbstractCommandListener?) {
        val params = Bundle(2)
        params.putInt(ExperimentalActions.EXTRA_RELAY_NUMBER, relayNumber)
        params.putBoolean(ExperimentalActions.EXTRA_IS_RELAY_ON, enabled)
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_SET_RELAY, params), listener)
    }

    /**
     * Move a servo to a particular pwm value
     *
     * @param channel the output channel the servo is attached to
     * @param pwm     PWM value to output to the servo. Servo’s generally accept pwm values between 1000 and 2000
     */
    fun setServo(channel: Int, pwm: Int) {
        setServo(channel, pwm, null)
    }

    /**
     * Move a servo to a particular pwm value
     *
     * @param channel  the output channel the servo is attached to
     * @param pwm      PWM value to output to the servo. Servo’s generally accept pwm values between 1000 and 2000
     * @param listener Register a callback to receive update of the command execution state.
     */
    fun setServo(channel: Int, pwm: Int, listener: AbstractCommandListener?) {
        val params = Bundle(2)
        params.putInt(ExperimentalActions.EXTRA_SERVO_CHANNEL, channel)
        params.putInt(ExperimentalActions.EXTRA_SERVO_PWM, pwm)
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_SET_SERVO, params), listener)
    }

    /**
     * Attempt to grab ownership and get a lock for the video stream. Can fail if
     * the video stream is already owned by another client.
     *
     * @param tag       Video tag.
     * @param callback  Video stream observer callback.
     *
     * @since 2.8.1
     */
    fun startVideoStream(tag: String, callback: IVideoStreamCallback?) {
        if (callback == null) {
            throw NullPointerException("Video stream callback can't be null")
        }
        capabilityChecker.checkFeatureSupport(CapabilityApi.FeatureIds.SOLO_VIDEO_STREAMING
        ) { featureId, result, resultInfo ->
            val listener: AbstractCommandListener = object : AbstractCommandListener() {
                override fun onSuccess() {
                    // Start VideoStreamObserver to connect to vehicle video stream and receive
                    // video stream packets.
                    videoStreamObserver.callback = callback
                    videoStreamObserver.start()
                    videoStreamObserver.callback?.onVideoStreamConnecting()
                }

                override fun onError(executionError: Int) {
                    videoStreamObserver.callback?.onError(executionError)
                }

                override fun onTimeout() {
                    videoStreamObserver.callback?.onTimeout()
                }
            }
            when (result) {
                CapabilityApi.FEATURE_SUPPORTED -> startVideoStreamForObserver(tag, listener)
                CapabilityApi.FEATURE_UNSUPPORTED -> postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                else -> postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
        }
    }

    /**
     * Release ownership of the video stream.
     *
     * @param tag   Video tag.
     *
     * @since 2.8.1
     */
    fun stopVideoStream(tag: String) {
        capabilityChecker.checkFeatureSupport(CapabilityApi.FeatureIds.SOLO_VIDEO_STREAMING
        ) { featureId, result, resultInfo ->
            val listener: AbstractCommandListener = object : AbstractCommandListener() {
                override fun onSuccess() {
                    videoStreamObserver.callback?.onVideoStreamDisconnecting()
                    videoStreamObserver.stop()
                }

                override fun onError(executionError: Int) {
                    videoStreamObserver.callback?.onError(executionError)
                }

                override fun onTimeout() {
                    videoStreamObserver.callback?.onTimeout()
                }
            }
            when (result) {
                CapabilityApi.FEATURE_SUPPORTED -> stopVideoStreamForObserver(tag, listener)
                CapabilityApi.FEATURE_UNSUPPORTED -> postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                else -> postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
        }
    }

    /**
     * Prepending 'observer' to the tag for differentiation
     * @param tag
     * @return
     */
    private fun getObserverTag(tag: String): String {
        return "observer" + if (TextUtils.isEmpty(tag)) "" else ".$tag"
    }

    /**
     * Attempt to grab ownership and start the video stream from the connected drone. Can fail if
     * the video stream is already owned by another client.
     *
     * @param tag       Video tag.
     * @param listener  Register a listener to receive update of the command execution status.
     * @since 2.8.1
     */
    private fun startVideoStreamForObserver(tag: String, listener: AbstractCommandListener) {
        val params = Bundle()
        params.putString(CameraActions.EXTRA_VIDEO_TAG, getObserverTag(tag))
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_START_VIDEO_STREAM_FOR_OBSERVER,
                params), listener)
    }

    /**
     * Stop the video stream from the connected drone, and release ownership.
     *
     * @param tag      Video tag.
     * @param listener Register a listener to receive update of the command execution status.
     * @since 2.8.1
     */
    private fun stopVideoStreamForObserver(tag: String, listener: AbstractCommandListener) {
        val params = Bundle()
        params.putString(CameraActions.EXTRA_VIDEO_TAG, getObserverTag(tag))
        drone.performAsyncActionOnDroneThread(Action(ExperimentalActions.ACTION_STOP_VIDEO_STREAM_FOR_OBSERVER, params),
                listener)
    }

    /**
     * Observer for vehicle video stream.
     */
    private class VideoStreamObserver(private val handler: Handler) : IpConnectionListener {
        private val TAG = VideoStreamObserver::class.java.simpleName
        private var linkConn: UdpConnection? = null
        private val onVideoStreamConnected: Runnable = object : Runnable {
            override fun run() {
                handler.removeCallbacks(this)
                if (callback != null) callback!!.onVideoStreamConnected()
            }
        }
        private val onVideoStreamDisconnected: Runnable = object : Runnable {
            override fun run() {
                handler.removeCallbacks(this)
                callback?.onVideoStreamDisconnected()
            }
        }
        var callback: IVideoStreamCallback? = null
            set
        private val reconnectTask: Runnable = object : Runnable {
            override fun run() {
                handler.removeCallbacks(this)
                linkConn?.connect()
            }
        }

        fun start() {
            if (linkConn == null) {
                linkConn = UdpConnection(handler, SOLO_STREAM_UDP_PORT,
                        UDP_BUFFER_SIZE, true, 42).also {
                    it.setIpConnectionListener(this)
                }
            }

            handler.removeCallbacks(reconnectTask)
            Log.d(TAG, "Connecting to video stream...")
            linkConn?.connect()
        }

        fun stop() {
            Log.d(TAG, "Stopping video manager")
            handler.removeCallbacks(reconnectTask)
            // Break the link
            linkConn?.disconnect()
            linkConn = null
        }

        override fun onIpConnected() {
            Log.d(TAG, "Connected to video stream")
            handler.post(onVideoStreamConnected)
            handler.removeCallbacks(reconnectTask)
        }

        override fun onIpDisconnected() {
            Log.d(TAG, "Video stream disconnected")
            handler.post(onVideoStreamDisconnected)
            handler.postDelayed(reconnectTask, RECONNECT_COUNTDOWN_IN_MILLIS)
        }

        override fun onPacketReceived(packetBuffer: ByteBuffer) {
            callback?.onAsyncVideoStreamPacketReceived(packetBuffer.array(), packetBuffer.limit())
        }

        companion object {
            private const val UDP_BUFFER_SIZE = 1500
            private const val RECONNECT_COUNTDOWN_IN_MILLIS = 1000L
            private const val SOLO_STREAM_UDP_PORT = 5600
        }
    }

    /**
     * Callback for directly observing video stream.
     */
    interface IVideoStreamCallback {
        /**
         * Invoked when opening the connection to the video stream endpoint
         */
        fun onVideoStreamConnecting()

        /**
         * Invoked when connected to the video stream endpoint
         */
        fun onVideoStreamConnected()

        /**
         * Invoked when closing the connection to the video stream endpoint
         */
        fun onVideoStreamDisconnecting()

        /**
         * Invoked when disconnected from the video stream endpoint
         */
        fun onVideoStreamDisconnected()

        /**
         * Invoked when detecting an error while connecting to the video stream endpoint
         * @param executionError
         */
        fun onError(executionError: Int)

        /**
         * Invoked when the connection to the video stream endpoint times out
         */
        fun onTimeout()

        /**
         * Invoked upon receipt of the video stream data packet.
         * This callback will be invoked on a background thread to avoid blocking the main thread while processing the received data
         * @param data      Video stream data packet
         * @param dataSize  Size of the video stream data
         */
        fun onAsyncVideoStreamPacketReceived(data: ByteArray?, dataSize: Int)
    }

    companion object {
        private val experimentalApiCache = ConcurrentHashMap<Drone, ExperimentalApi>()
        private val apiBuilder: Builder<ExperimentalApi> = Builder { drone -> ExperimentalApi(drone) }

        /**
         * Retrieves an ExperimentalApi instance.
         *
         * @param drone target vehicle.
         * @return a ExperimentalApi instance.
         */
        @JvmStatic
        fun getApi(drone: Drone?): ExperimentalApi {
            return getApi(drone, experimentalApiCache, apiBuilder)
        }
    }
}
