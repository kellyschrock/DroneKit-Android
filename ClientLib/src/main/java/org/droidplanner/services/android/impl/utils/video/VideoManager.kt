package org.droidplanner.services.android.impl.utils.video

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.RemoteException
import android.text.TextUtils
import android.util.Log
import android.view.Surface
import com.o3dr.android.client.utils.connection.AbstractIpConnection
import com.o3dr.android.client.utils.connection.IpConnectionListener
import com.o3dr.android.client.utils.connection.UdpConnection
import com.o3dr.android.client.utils.video.DecoderListener
import com.o3dr.android.client.utils.video.MediaCodecManager
import com.o3dr.services.android.lib.drone.action.CameraActions
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.model.ICommandListener
import timber.log.Timber
import java.io.IOException
import java.nio.ByteBuffer
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/**
 * Handles the video stream from artoo.
 */
class VideoManager(context: Context?, private val handler: Handler) : IpConnectionListener {
    private val videoStreamObserverUsed = AtomicBoolean(false)

    interface LinkListener {
        fun onLinkConnected()
        fun onLinkDisconnected()
    }

    private val reconnectTask: Runnable = object : Runnable {
        override fun run() {
            handler.removeCallbacks(this)
            if (linkConn != null) linkConn!!.connect()
        }
    }
    private var linkListener: LinkListener? = null
    private val isStarted = AtomicBoolean(false)
    private val wasConnected = AtomicBoolean(false)
    private val videoOwnerId = AtomicReference(NO_VIDEO_OWNER)
    private val videoTagRef = AtomicReference("")
    protected var linkConn: UdpConnection? = null
    private val mediaCodecManager: MediaCodecManager = MediaCodecManager(handler)
    private val streamRecorder: StreamRecorder = StreamRecorder(context!!)
    private var linkPort = -1
    private fun enableLocalRecording(filename: String) {
        streamRecorder.enableRecording(filename)
    }

    private fun disableLocalRecording() {
        streamRecorder.disableRecording()
    }

    fun startDecoding(udpIP: String?, udpPort: Int, surface: Surface, listener: DecoderListener?) {
        start(udpIP, udpPort, null)
        val currentSurface = mediaCodecManager.surface
        if (surface === currentSurface) {
            listener?.onDecodingStarted()
            return
        }

        // Stop any in progress decoding.
        Log.i(TAG, "Setting up video stream decoding.")
        mediaCodecManager.stopDecoding(object : DecoderListener {
            override fun onDecodingStarted() {}
            override fun onDecodingError() {}
            override fun onDecodingEnded() {
                try {
                    Log.i(TAG, "Video decoding set up complete. Starting...")
                    mediaCodecManager.startDecoding(surface, listener)
                } catch (e: IOException) {
                    Log.e(TAG, "Unable to create media codec.", e)
                    listener?.onDecodingError()
                } catch (e: IllegalStateException) {
                    Log.e(TAG, "Unable to create media codec.", e)
                    listener?.onDecodingError()
                }
            }
        })
    }

    fun reset() {
        Timber.d("Resetting video tag (%s) and owner id (%s)", videoTagRef.get(), videoOwnerId.get())
        videoTagRef.set("")
        videoOwnerId.set(NO_VIDEO_OWNER)
        disableLocalRecording()
        stopDecoding(null)
    }

    fun stopDecoding(listener: DecoderListener?) {
        Log.i(TAG, "Aborting video decoding process.")
        mediaCodecManager.stopDecoding(listener)
        stop()
    }

    val isLinkConnected: Boolean
        get() = linkConn != null && linkConn!!.getConnectionStatus() == AbstractIpConnection.STATE_CONNECTED

    private fun start(udpIP: String?, udpPort: Int, listener: LinkListener?) {
        if (linkConn == null || udpPort != linkPort) {
            if (isStarted.get()) {
                stop()
            }
            try {
                linkConn = if (udpIP != null) UdpConnection(handler, udpIP, udpPort, UDP_BUFFER_SIZE, true, 42) else UdpConnection(handler, udpPort, UDP_BUFFER_SIZE, true, 42)
                linkConn!!.setIpConnectionListener(this)
                linkPort = udpPort
            } catch (ex: Throwable) {
                Timber.e(ex, ex.message)
                listener!!.onLinkDisconnected()
                handler!!.removeCallbacks(reconnectTask)
                return
            }
        }
        Log.d(TAG, "Starting video manager")
        handler!!.removeCallbacks(reconnectTask)
        isStarted.set(true)
        streamRecorder.startConverterThread()
        linkConn!!.connect()
        linkListener = listener
    }

    private fun stop() {
        Log.d(TAG, "Stopping video manager")
        handler!!.removeCallbacks(reconnectTask)
        isStarted.set(false)
        if (linkConn != null) {
            //Break the link
            linkConn!!.disconnect()
            linkConn = null
        }
        linkPort = -1
        streamRecorder.stopConverterThread()
    }

    override fun onIpConnected() {
        Log.d(TAG, "Connected to video stream")
        handler!!.removeCallbacks(reconnectTask)
        wasConnected.set(true)
        if (linkListener != null) linkListener!!.onLinkConnected()
    }

    override fun onIpDisconnected() {
        Log.d(TAG, "Video stream disconnected")
        if (isStarted.get()) {
            if (shouldReconnect()) {
                //Try to reconnect
                handler!!.postDelayed(reconnectTask, RECONNECT_COUNTDOWN)
            }
            if (linkListener != null && wasConnected.get()) linkListener!!.onLinkDisconnected()
            wasConnected.set(false)
        }
    }

    override fun onPacketReceived(packetBuffer: ByteBuffer) {
        if (!videoStreamObserverUsed.get()) {
            // Feed this data stream to the decoder.
            mediaCodecManager.onInputDataReceived(packetBuffer.array(), packetBuffer.limit())
        }
    }

    protected fun postSuccessEvent(listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onSuccess()
                } catch (e: RemoteException) {
                    Log.e(TAG, e.message, e)
                }
            })
        }
    }

    protected fun postTimeoutEvent(listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onTimeout()
                } catch (e: RemoteException) {
                    Log.e(TAG, e.message, e)
                }
            })
        }
    }

    protected fun postErrorEvent(error: Int, listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onError(error)
                } catch (e: RemoteException) {
                    Log.e(TAG, e.message, e)
                }
            })
        }
    }

    protected fun shouldReconnect(): Boolean {
        return true
    }

    private fun checkForLocalRecording(appId: String, videoProps: Bundle) {
        if (TextUtils.isEmpty(appId)) return
        val isLocalRecordingEnabled = videoProps.getBoolean(CameraActions.EXTRA_VIDEO_ENABLE_LOCAL_RECORDING)
        if (isLocalRecordingEnabled) {
            var localRecordingFilename = videoProps.getString(CameraActions.EXTRA_VIDEO_LOCAL_RECORDING_FILENAME)
            if (TextUtils.isEmpty(localRecordingFilename)) {
                localRecordingFilename = appId + "." + FILE_DATE_FORMAT.format(Date())
            }
            if (!localRecordingFilename.equals(streamRecorder.getRecordingFilename(), ignoreCase = true)) {
                if (streamRecorder.isRecordingEnabled) {
                    disableLocalRecording()
                }
                enableLocalRecording(localRecordingFilename)
            }
        } else {
            disableLocalRecording()
        }
    }

    fun startVideoStream(videoProps: Bundle, appId: String, newVideoTag: String?, videoSurface: Surface?,
                         listener: ICommandListener?) {
        var newVideoTag = newVideoTag
        Timber.d("Video stream start request from %s. Video owner is %s.", appId, videoOwnerId.get())
        if (!isAppIdValid(appId, listener)) {
            return
        }
        val udpPort = videoProps.getInt(CameraActions.EXTRA_VIDEO_PROPS_UDP_PORT, -1)
        if (videoSurface == null || udpPort == -1) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }

        // UDP IP is optional. May not actually be necessary.
        val udpIP = videoProps.getString(CameraActions.EXTRA_VIDEO_PROPS_UDP_IP, null)
        if (newVideoTag == null) newVideoTag = ""
        if (appId == videoOwnerId.get()) {
            var currentVideoTag = videoTagRef.get()
            if (currentVideoTag == null) currentVideoTag = ""
            if (newVideoTag == currentVideoTag) {
                // Check if the local recording state needs to be updated.
                checkForLocalRecording(appId, videoProps)
                postSuccessEvent(listener)
                return
            }
        }
        if (videoOwnerId.compareAndSet(NO_VIDEO_OWNER, appId)) {
            videoTagRef.set(newVideoTag)
            checkForLocalRecording(appId, videoProps)
            Timber.i("Starting video decoding.")
            startDecoding(udpIP, udpPort, videoSurface, object : DecoderListener {
                override fun onDecodingStarted() {
                    Timber.i("Video decoding started.")
                    postSuccessEvent(listener)
                }

                override fun onDecodingError() {
                    Timber.i("Video decoding failed.")
                    postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                    reset()
                }

                override fun onDecodingEnded() {
                    Timber.i("Video decoding ended successfully.")
                    reset()
                }
            })
        } else {
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
        }
    }

    fun startVideoStreamForObserver(appId: String, newVideoTag: String?,
                                    listener: ICommandListener?) {
        var newVideoTag = newVideoTag
        Timber.d("Video stream start request from %s. Video owner is %s.", appId,
                videoOwnerId.get())
        if (!isAppIdValid(appId, listener)) {
            return
        }
        if (newVideoTag == null) newVideoTag = ""
        if (appId == videoOwnerId.get()) {
            var currentVideoTag = videoTagRef.get()
            if (currentVideoTag == null) currentVideoTag = ""
            if (newVideoTag == currentVideoTag) {
                postSuccessEvent(listener)
                return
            }
        }
        if (videoOwnerId.compareAndSet(NO_VIDEO_OWNER, appId)) {
            videoTagRef.set(newVideoTag)
            Timber.i("Successful lock obtained for app with id %s.", appId)
            videoStreamObserverUsed.set(true)
            postSuccessEvent(listener)
        } else {
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
        }
    }

    fun stopVideoStream(appId: String, currentVideoTag: String?,
                        listener: ICommandListener?) {
        var currentVideoTag = currentVideoTag
        Timber.d("Video stream stop request from %s. Video owner is %s.", appId, videoOwnerId.get())
        if (!isAppIdValid(appId, listener)) {
            return
        }
        val currentVideoOwner = videoOwnerId.get()
        if (NO_VIDEO_OWNER == currentVideoOwner) {
            Timber.d("No video owner set. Nothing to do.")
            disableLocalRecording()
            postSuccessEvent(listener)
            return
        }
        if (currentVideoTag == null) currentVideoTag = ""
        if (appId == currentVideoOwner && currentVideoTag == videoTagRef.get() && videoOwnerId.compareAndSet(currentVideoOwner, NO_VIDEO_OWNER)) {
            videoTagRef.set("")
            disableLocalRecording()
            Timber.d("Stopping video decoding. Current owner is %s.", currentVideoOwner)
            Timber.i("Stopping video decoding.")
            stopDecoding(object : DecoderListener {
                override fun onDecodingStarted() {}
                override fun onDecodingError() {
                    postSuccessEvent(listener)
                }

                override fun onDecodingEnded() {
                    postSuccessEvent(listener)
                }
            })
        } else {
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
        }
    }

    fun stopVideoStreamForObserver(appId: String, currentVideoTag: String?,
                                   listener: ICommandListener?) {
        var currentVideoTag = currentVideoTag
        Timber.d("Video stream stop request from %s. Video owner is %s.", appId, videoOwnerId.get())
        if (!isAppIdValid(appId, listener)) {
            return
        }
        val currentVideoOwner = videoOwnerId.get()
        if (NO_VIDEO_OWNER == currentVideoOwner) {
            Timber.d("No video owner set. Nothing to do.")
            postSuccessEvent(listener)
            return
        }
        if (currentVideoTag == null) currentVideoTag = ""
        if (appId == currentVideoOwner && currentVideoTag == videoTagRef.get() && videoOwnerId.compareAndSet(currentVideoOwner, NO_VIDEO_OWNER)) {
            videoTagRef.set("")
            Timber.d("Stopping video decoding. Current owner is %s.", currentVideoOwner)
            Timber.i("Stop using video observer...")
            videoStreamObserverUsed.set(false)
            postSuccessEvent(listener)
        } else {
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
        }
    }

    fun tryStoppingVideoStream(parentId: String) {
        if (TextUtils.isEmpty(parentId)) return
        val videoOwner = videoOwnerId.get()
        if (NO_VIDEO_OWNER == videoOwner) return
        if (videoOwner == parentId) {
            Timber.d("Stopping video owned by %s", parentId)
            if (videoStreamObserverUsed.get()) {
                stopVideoStreamForObserver(parentId, videoTagRef.get(), null)
            } else {
                stopVideoStream(parentId, videoTagRef.get(), null)
            }
        }
    }

    private fun isAppIdValid(appId: String, listener: ICommandListener?): Boolean {
        if (TextUtils.isEmpty(appId)) {
            Timber.w("Owner id is empty.")
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
            return false
        }
        return true
    }

    companion object {
        private val TAG = VideoManager::class.java.simpleName
        private val FILE_DATE_FORMAT = SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", Locale.US)
        private const val NO_VIDEO_OWNER = "no_video_owner"
        protected const val RECONNECT_COUNTDOWN = 1000L //ms
        const val ARTOO_UDP_PORT = 5600
        private const val UDP_BUFFER_SIZE = 1500
    }

    init {
        mediaCodecManager.setNaluChunkListener(streamRecorder)
    }
}
