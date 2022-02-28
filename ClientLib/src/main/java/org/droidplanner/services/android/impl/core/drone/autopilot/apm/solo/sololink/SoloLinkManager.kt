package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.sololink

import android.content.Context
import android.os.Handler
import android.text.TextUtils
import com.o3dr.android.client.BuildConfig
import com.o3dr.android.client.utils.connection.TcpConnection
import com.o3dr.android.client.utils.connection.UdpConnection
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonTypes
import com.o3dr.services.android.lib.drone.companion.solo.tlv.*
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.SimpleCommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.AbstractLinkManager
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.controller.ControllerLinkManager
import org.droidplanner.services.android.impl.utils.connection.SshConnection
import timber.log.Timber
import java.io.IOException
import java.nio.ByteBuffer
import java.util.*
import java.util.concurrent.ExecutorService
import java.util.concurrent.atomic.AtomicReference

/**
 * Handles solo link related logic.
 */
class SoloLinkManager(context: Context?, private val serverIp: String, handler: Handler?, asyncExecutor: ExecutorService?)
    : AbstractLinkManager<SoloLinkListener>(context, TcpConnection(handler, serverIp, SOLO_LINK_TCP_PORT), handler, asyncExecutor) {

    private val followDataConn: UdpConnection?
    private val sshLink: SshConnection = SshConnection(serverIp, SoloComp.SSH_USERNAME, SoloComp.SSH_PASSWORD)
    private val presetButtonAGetter = SoloButtonSettingGetter(ButtonTypes.BUTTON_A, ButtonTypes.BUTTON_EVENT_PRESS)
    private val presetButtonBGetter = SoloButtonSettingGetter(ButtonTypes.BUTTON_B, ButtonTypes.BUTTON_EVENT_PRESS)
    private val goproStateGetter = SoloGoproRequestState()
    private val loadedPresetButtonA = AtomicReference<SoloButtonSetting>()
    private val loadedPresetButtonB = AtomicReference<SoloButtonSetting>()
    private val vehicleVersion = AtomicReference("")
    private val pixhawkVersion = AtomicReference("")
    private val gimbalVersion = AtomicReference("")

    private val soloLinkVersionRetriever = Runnable {
        val version = retrieveVersion(SOLO_VERSION_FILENAME)
        if (version != null) {
            vehicleVersion.set(version)
        }

        if (areVersionsSet()) {
            linkListener?.onVersionsUpdated()
        }
    }

    private val pixhawkVersionRetriever = Runnable {
        val version = retrieveVersion(PIXHAWK_VERSION_FILENAME)
        if (version != null) {
            pixhawkVersion.set(version)
        }

        if (areVersionsSet()) {
            linkListener?.onVersionsUpdated()
        }
    }

    private val gimbalVersionRetriever = Runnable {
        val version = retrieveVersion(GIMBAL_VERSION_FILENAME)
        if (version != null) gimbalVersion.set(version)
        if (areVersionsSet()) {
            linkListener?.onVersionsUpdated()
        }
    }

    private var linkListener: SoloLinkListener? = null

    val soloLinkIp: String
        get() = if (BuildConfig.SITL_DEBUG) BuildConfig.SOLO_LINK_IP else serverIp

    fun getVehicleVersion(): String {
        return vehicleVersion.get()
    }

    fun getPixhawkVersion(): String {
        return pixhawkVersion.get()
    }

    fun getGimbalVersion(): String {
        return gimbalVersion.get()
    }

    fun areVersionsSet(): Boolean {
        return !TextUtils.isEmpty(vehicleVersion.get()) && !TextUtils.isEmpty(pixhawkVersion.get())
    }

    override fun start(listener: SoloLinkListener) {
        if (!isStarted) {
            Timber.i("Starting solo link manager")
        }
        super.start(listener)
        this.linkListener = listener
    }

    override fun stop() {
        if (isStarted) {
            Timber.i("Stopping solo link manager")
        }
        super.stop()
    }

    override fun refreshState() {
        Timber.d("Connected to sololink.")

        //Load the mac address for the vehicle.
        loadMacAddress()
        loadPresetButtonSettings()
        loadGoproState()
        refreshSoloLinkVersions()
    }

    override fun getSshLink(): SshConnection {
        return sshLink
    }

    override fun onIpDisconnected() {
        Timber.d("Disconnected from sololink.")
        super.onIpDisconnected()
    }

    override fun onPacketReceived(packetBuffer: ByteBuffer) {
        val tlvMsgs = TLVMessageParser.parseTLVPacket(packetBuffer)
        if (tlvMsgs.isEmpty()) {
            return
        }
        for (tlvMsg in tlvMsgs) {
            val messageType = tlvMsg.messageType
            Timber.d("Received tlv message: $messageType")
            when (messageType) {
                TLVMessageTypes.TYPE_SOLO_MESSAGE_SHOT_MANAGER_ERROR -> Timber.w((tlvMsg as SoloMessageShotManagerError).exceptionInfo)
                TLVMessageTypes.TYPE_SOLO_GET_BUTTON_SETTING -> {
                    val receivedPresetButton = tlvMsg as SoloButtonSettingGetter
                    handleReceivedPresetButton(receivedPresetButton)
                }
            }

            linkListener?.onTlvPacketReceived(tlvMsg)
        }
    }

    private fun sendPacket(payload: ByteArray, payloadSize: Int, listener: ICommandListener?) {
        linkConn.sendPacket(payload, payloadSize, listener!!)
    }

    private fun sendFollowPacket(payload: ByteArray, payloadSize: Int, listener: ICommandListener?) {
        checkNotNull(followDataConn) { "Unable to send follow data." }
        followDataConn.sendPacket(payload, payloadSize, listener!!)
    }

    fun sendTLVPacket(packet: TLVPacket?, listener: ICommandListener?) {
        sendTLVPacket(packet, false, listener)
    }

    fun sendTLVPacket(packet: TLVPacket?, useFollowLink: Boolean, listener: ICommandListener?) {
        if (packet == null) {
            return
        }
        val messagePayload = packet.toBytes()
        if (useFollowLink) {
            sendFollowPacket(messagePayload, messagePayload.size, listener)
        } else {
            sendPacket(messagePayload, messagePayload.size, listener)
        }
    }

    fun loadPresetButtonSettings() {
        sendTLVPacket(presetButtonAGetter, object : SimpleCommandListener() {
            override fun onSuccess() {
                sendTLVPacket(presetButtonBGetter, null)
            }
        })
    }

    private fun loadGoproState() {
        sendTLVPacket(goproStateGetter, null)
    }

    private fun handleReceivedPresetButton(presetButton: SoloButtonSetting) {
        val buttonType = presetButton.button
        when (buttonType) {
            ButtonTypes.BUTTON_A -> {
                loadedPresetButtonA.set(presetButton)
                linkListener?.onPresetButtonLoaded(buttonType, presetButton)
            }
            ButtonTypes.BUTTON_B -> {
                loadedPresetButtonB.set(presetButton)
                linkListener?.onPresetButtonLoaded(buttonType, presetButton)
            }
        }
    }

    fun getLoadedPresetButton(buttonType: Int): SoloButtonSetting? {
        return when (buttonType) {
            ButtonTypes.BUTTON_A -> loadedPresetButtonA.get()
            ButtonTypes.BUTTON_B -> loadedPresetButtonB.get()
            else -> null
        }
    }

    /**
     * Update the vehicle preset button settings
     */
    fun pushPresetButtonSettings(buttonSetter: SoloButtonSettingSetter?, listener: ICommandListener?) {
        if (!isLinkConnected || buttonSetter == null) {
            return
        }
        sendTLVPacket(buttonSetter, object : SimpleCommandListener() {
            override fun onSuccess() {
                postSuccessEvent(listener)
                handleReceivedPresetButton(buttonSetter)
            }

            override fun onTimeout() {
                postTimeoutEvent(listener)
            }
        })
    }

    fun disableFollowDataConnection() {
        Timber.d("disableFollowDataConnection(): followDataConn=%s", followDataConn)
        followDataConn?.disconnect()
    }

    fun enableFollowDataConnection() {
        Timber.d("enableFollowDataConnection(): followDataConn=%s", followDataConn)
        followDataConn?.connect()
    }

    fun updateSololinkWifi(wifiSsid: CharSequence, password: CharSequence): Boolean {
        Timber.d(String.format(Locale.US, "Updating solo wifi ssid to %s with password %s", wifiSsid, password))
        return try {
            val ssidUpdateResult = sshLink.execute(ControllerLinkManager.SOLOLINK_SSID_CONFIG_PATH + " --set-wifi-ssid " +
                    wifiSsid)
            val passwordUpdateResult = sshLink.execute(ControllerLinkManager.SOLOLINK_SSID_CONFIG_PATH + " --set-wifi-password " +
                    password)
            val restartResult = sshLink.execute(ControllerLinkManager.SOLOLINK_SSID_CONFIG_PATH + " --reboot")
            true
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while updating the sololink wifi ssid.")
            false
        }
    }

    private fun updateSoloLinkVersion() {
        postAsyncTask(soloLinkVersionRetriever)
    }

    private fun updatePixhawkVersion() {
        postAsyncTask(pixhawkVersionRetriever)
    }

    private fun updateGimbalVersion() {
        postAsyncTask(gimbalVersionRetriever)
    }

    private fun retrieveVersion(versionFile: String): String? {
        try {
            val version = sshLink.execute("cat $versionFile")
            return if (TextUtils.isEmpty(version)) {
                Timber.d("No version file was found")
                ""
            } else {
                version!!.split("\n".toRegex()).toTypedArray()[0]
            }
        } catch (e: IOException) {
            Timber.e("Unable to retrieve the current version.", e)
        }
        return null
    }

    /**
     * Refresh the vehicle's components versions
     */
    fun refreshSoloLinkVersions() {
        updateSoloLinkVersion()
        updatePixhawkVersion()
        updateGimbalVersion()
    }

    companion object {
        const val SOLO_LINK_IP = "10.1.1.10"
        const val SOLO_LINK_TCP_PORT = 5507
        private const val SHOT_FOLLOW_UDP_PORT = 14558
        private const val SOLO_VERSION_FILENAME = "/VERSION"
        private const val PIXHAWK_VERSION_FILENAME = "/PIX_VERSION"
        private const val GIMBAL_VERSION_FILENAME = "/AXON_VERSION"
    }

    init {
        var dataConn: UdpConnection? = null
        try {
            dataConn = UdpConnection(handler, soloLinkIp, SHOT_FOLLOW_UDP_PORT, 14557)
            Timber.d("Created Follow UDP connection on ports: %d, %d", SHOT_FOLLOW_UDP_PORT, 14557)
        } catch (e: Throwable) {
            Timber.e(e, "Error while creating follow udp connection.")
        }
        followDataConn = dataConn
    }
}
