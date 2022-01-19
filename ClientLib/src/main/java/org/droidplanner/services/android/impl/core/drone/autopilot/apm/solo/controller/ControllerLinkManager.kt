package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.controller

import android.content.Context
import android.os.Handler
import android.text.TextUtils
import android.util.Pair
import com.github.zafarkhaja.semver.Version
import com.o3dr.android.client.utils.TxPowerComplianceCountries.Companion.defaultCountry
import com.o3dr.android.client.utils.connection.IpConnectionListener
import com.o3dr.android.client.utils.connection.TcpConnection
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonPacket.Companion.parseButtonPacket
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVMessageParser
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.AbstractLinkManager
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp
import org.droidplanner.services.android.impl.utils.NetworkUtils
import org.droidplanner.services.android.impl.utils.connection.SshConnection
import timber.log.Timber
import java.io.IOException
import java.nio.ByteBuffer
import java.util.*
import java.util.concurrent.ExecutorService
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference

/**
 * Handles artoo link related logic.
 */
class ControllerLinkManager(context: Context?, handler: Handler, asyncExecutor: ExecutorService?)
    : AbstractLinkManager<ControllerLinkListener>(context, TcpConnection(handler, ARTOO_IP, ARTOO_BUTTON_PORT), handler, asyncExecutor) {

    private val _controllerVersion = AtomicReference("")
    private val _stm32Version = AtomicReference("")
    private val txPowerCompliantCountry = AtomicReference(defaultCountry.name)
    private val controllerMode = AtomicInteger(SoloControllerMode.UNKNOWN_MODE)
    private val controllerUnits = AtomicReference("")
    private val sololinkWifiInfo = AtomicReference(Pair.create("", ""))
    private val isVideoHandshakeStarted = AtomicBoolean(false)
    private val isBatteryStarted = AtomicBoolean(false)
    private val videoHandshake: TcpConnection = TcpConnection(handler, ARTOO_IP, ARTOO_VIDEO_HANDSHAKE_PORT)
    private val batteryConnection: TcpConnection = TcpConnection(handler, ARTOO_IP, ARTOO_BATTERY_PORT)

    private val reconnectBatteryTask: Runnable = object : Runnable {
        override fun run() {
            handler.removeCallbacks(this)
            batteryConnection.connect()
        }
    }

    private val reconnectVideoHandshake: Runnable = object : Runnable {
        override fun run() {
            handler.removeCallbacks(this)
            videoHandshake.connect()
        }
    }

    private val artooVersionRetriever = Runnable {
        val version = retrieveVersion(ARTOO_VERSION_FILENAME)
        if (version != null) _controllerVersion.set(version)
        updateControllerModeIfPossible()
        updateControllerUnitIfPossible()
        onVersionsUpdated()
    }

    private val stm32VersionRetriever = Runnable {
        val version = retrieveVersion(STM32_VERSION_FILENAME)
        if (version != null) _stm32Version.set(version)
        onVersionsUpdated()
    }

    private val loadWifiInfo = Runnable {
        try {
            val wifiName = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-wifi-ssid")
            val wifiPassword = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-wifi-password")
            if (!TextUtils.isEmpty(wifiName) && !TextUtils.isEmpty(wifiPassword)) {
                val wifiInfo = Pair.create(wifiName.trim { it <= ' ' }, wifiPassword.trim { it <= ' ' })
                sololinkWifiInfo.set(wifiInfo)
                if (linkListener != null) linkListener!!.onWifiInfoUpdated(wifiInfo.first, wifiInfo.second)
            }
        } catch (e: IOException) {
            Timber.e(e, "Unable to retrieve sololink wifi info.")
        }
    }

    private val checkEUTxPowerCompliance = Runnable {
        var compliantCountry: String
        try {
            compliantCountry = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-wifi-country").trim { it <= ' ' }
            if (linkListener != null) linkListener!!.onTxPowerComplianceCountryUpdated(compliantCountry)
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while querying wifi country.")
            compliantCountry = defaultCountry.name
        }
        txPowerCompliantCountry.set(compliantCountry)
    }

    private val artooModeRetriever = Runnable {
        Timber.i("Retrieving controller mode")
        try {
            val response = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-ui-mode")
            val trimmedResponse = if (TextUtils.isEmpty(response)) "" else response.trim { it <= ' ' }
            when (trimmedResponse) {
                "1" -> setControllerMode(SoloControllerMode.MODE_1)
                "2" -> setControllerMode(SoloControllerMode.MODE_2)
                else -> {
                    Timber.w("Unable to parse received controller mode.")
                    setControllerMode(SoloControllerMode.UNKNOWN_MODE)
                }
            }
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while getting controller mode.")
        }
    }

    private val unitsRetriever = Runnable {
        Timber.d("Retrieving controller units.")
        try {
            val response = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-ui-units")
            @ControllerUnit val trimmedResponse = if (TextUtils.isEmpty(response)) SoloControllerUnits.UNKNOWN else response.trim { it <= ' ' }
            when (trimmedResponse) {
                SoloControllerUnits.METRIC, SoloControllerUnits.IMPERIAL, SoloControllerUnits.UNKNOWN -> controllerUnit = trimmedResponse
                else -> Timber.w("Received unknown value for controller unit: %s", trimmedResponse)
            }
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while retrieving the controller units.")
        }
    }

    private var linkListener: ControllerLinkListener? = null
    private val streamingPermission = AtomicBoolean(false)

    fun hasStreamingPermission(): Boolean {
        return streamingPermission.get()
    }

    fun areVersionsSet(): Boolean {
        return !TextUtils.isEmpty(_controllerVersion.get()) && !TextUtils.isEmpty(_stm32Version.get())
    }

    /**
     * @return the controller version.
     */
    val controllerVersion: String
        get() = _controllerVersion.get()

    /**
     * @return the stm32 version
     */
    fun getStm32Version(): String {
        return _stm32Version.get()
    }

    /**
     * @return the country the controller is compliant with tx power levels.
     */
    fun getTxPowerCompliantCountry(): String {
        return txPowerCompliantCountry.get()
    }

    /**
     * Return the current controller mode
     *
     * @return MODE_1 or MODE_2
     */
    @ControllerMode
    fun getControllerMode(): Int {
        return controllerMode.get()
    }

    /**
     * Return the current controller unit
     *
     * @return @see [SoloControllerUnits.ControllerUnit]
     */
    @get:ControllerUnit
    var controllerUnit: String
        get() = controllerUnits.get()
        private set(unit) {
            controllerUnits.set(unit)
            if (linkListener != null) linkListener!!.onControllerUnitUpdated(unit)
        }

    private fun startVideoManager() {
        handler.removeCallbacks(reconnectVideoHandshake)
        isVideoHandshakeStarted.set(true)
        videoHandshake.connect()
    }

    private fun stopVideoManager() {
        handler.removeCallbacks(reconnectVideoHandshake)
        isVideoHandshakeStarted.set(false)
        videoHandshake.disconnect()
    }

    private fun loadSololinkWifiInfo() {
        postAsyncTask(loadWifiInfo)
    }

    fun updateSololinkWifi(wifiSsid: CharSequence, password: CharSequence): Boolean {
        Timber.d(String.format(Locale.US, "Updating artoo wifi ssid to %s with password %s", wifiSsid, password))
        return try {
            val ssidUpdateResult = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --set-wifi-ssid " + wifiSsid)
            val passwordUpdateResult = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --set-wifi-password " +
                    password)
            restartController()
            true
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while updating the sololink wifi ssid.")
            false
        }
    }

    val wifiSettings: Pair<String, String>
        get() = sololinkWifiInfo.get()

    override fun start(listener: ControllerLinkListener) {
        this.linkListener = listener
        if (!isStarted) {
            Timber.i("Starting artoo link manager")
        }
        super.start(listener)

        //TODO: update when battery info is available
//        handler.removeCallbacks(reconnectBatteryTask);
        //isBatteryStarted.set(true);
        //batteryConnection.connect();
    }

    override fun stop() {
        if (isStarted) {
            Timber.i("Stopping artoo link manager")
        }

        //TODO: update when battery info is available
        /*handler.removeCallbacks(reconnectBatteryTask);
        isBatteryStarted.set(false);
        batteryConnection.disconnect();*/super.stop()
    }

    override fun isLinkConnected(): Boolean {
        return NetworkUtils.isOnSololinkNetwork(context)
    }

    override fun refreshState() {
        Timber.d("Artoo link connected.")

        //Load the mac address for the vehicle.
        loadMacAddress()
        startVideoManager()

        //Update sololink wifi
        loadSololinkWifiInfo()
        refreshControllerVersions()

        //Update the tx power compliance
        loadCurrentEUTxPowerComplianceMode()
    }

    override fun getSshLink(): SshConnection {
        return Companion.sshLink
    }

    private fun onVersionsUpdated() {
        if (linkListener != null && areVersionsSet()) linkListener!!.onVersionsUpdated()
    }

    private fun updateControllerUnitIfPossible() {
        if (doesSupportControllerMode()) {
            Timber.d("Updating current controller unit.")
            loadControllerUnit()
        } else {
            Timber.w("This controller version doesn't support controller unit retrieval.")
        }
    }

    private fun updateControllerModeIfPossible() {
        if (doesSupportControllerMode()) {
            //load current controller mode
            Timber.d("Updating current controller mode.")
            loadCurrentControllerMode()
        } else {
            Timber.w("This controller version doesn't support controller mode retrieval.")
        }
    }

    private fun doesSupportControllerMode(): Boolean {
        val version = _controllerVersion.get()
        return if (TextUtils.isEmpty(version)) false else try {
            val currentVersion = Version.valueOf(version)
            CONTROLLER_MODE_MIN_VERSION.lessThanOrEqualTo(currentVersion)
        } catch (e: Exception) {
            Timber.e(e, "Unable to parse controller version.")
            false
        }
    }

    override fun onIpDisconnected() {
        Timber.d("Artoo link disconnected.")
        stopVideoManager()
        super.onIpDisconnected()
    }

    override fun onPacketReceived(packetBuffer: ByteBuffer) {
        val buttonPacket = parseButtonPacket(packetBuffer) ?: return
        val buttonId = buttonPacket.buttonId.toInt()
        Timber.d("Button pressed: $buttonId")
        if (linkListener != null) linkListener!!.onButtonPacketReceived(buttonPacket)
    }

    private fun updateArtooVersion() {
        postAsyncTask(artooVersionRetriever)
    }

    private fun updateStm32Version() {
        postAsyncTask(stm32VersionRetriever)
    }

    private fun retrieveVersion(versionFile: String): String? {
        try {
            val version = Companion.sshLink.execute("cat $versionFile")
            return if (TextUtils.isEmpty(version)) {
                Timber.d("No version file was found")
                ""
            } else {
                version.split("\n".toRegex()).toTypedArray()[0]
            }
        } catch (e: IOException) {
            Timber.e("Unable to retrieve the current version.", e)
        }
        return null
    }

    fun updateControllerUnit(@ControllerUnit unit: String, listener: ICommandListener?) {
        postAsyncTask(Runnable {
            val supportControllerMode = doesSupportControllerMode()
            if (!supportControllerMode) {
                postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                return@Runnable
            }
            Timber.d("Switching controller unit to %s", unit)
            try {
                val command = SOLOLINK_SSID_CONFIG_PATH + " --set-ui-units %s"
                val response = Companion.sshLink.execute(String.format(Locale.US, command, unit))
                Timber.d("Response from unit change was: %s", response)
                postSuccessEvent(listener)
                controllerUnit = unit
            } catch (e: IOException) {
                Timber.e(e, "Error occurred while changing controller unit.")
                postTimeoutEvent(listener)
            }
        })
    }

    fun updateControllerMode(@ControllerMode mode: Int, listener: ICommandListener?) {
        postAsyncTask {
            Timber.d("Switching controller to mode %d", mode)
            try {
                val supportControllerMode = doesSupportControllerMode()
                val command = if (supportControllerMode) SOLOLINK_SSID_CONFIG_PATH + " --set-ui-mode %d" else "runStickMapperMode%d.sh"
                val response: String
                when (mode) {
                    SoloControllerMode.MODE_1 -> {
                        response = Companion.sshLink.execute(String.format(Locale.US, command, mode))
                        postSuccessEvent(listener)
                    }
                    SoloControllerMode.MODE_2 -> {
                        response = Companion.sshLink.execute(String.format(Locale.US, command, mode))
                        postSuccessEvent(listener)
                    }
                    else -> {
                        response = "No response."
                        postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
                    }
                }
                Timber.d("Response from switch mode command was: %s", response)
                if (supportControllerMode) {
                    setControllerMode(mode)
                }
            } catch (e: IOException) {
                Timber.e(e, "Error occurred while changing controller modes.")
                postTimeoutEvent(listener)
            }
        }
    }

    private fun setControllerMode(@ControllerMode mode: Int) {
        controllerMode.set(mode)
        if (linkListener != null) linkListener!!.onControllerModeUpdated()
    }

    fun setTxPowerComplianceCountry(compliantCountry: String, listener: ICommandListener?) {
        postAsyncTask {
            Timber.d("Enabling %s Tx power compliance mode", compliantCountry)
            try {
                val currentCompliance = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --get-wifi-country").trim { it <= ' ' }
                if (currentCompliance != compliantCountry) {
                    val response: String
                    response = Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --set-wifi-country " + compliantCountry + "; echo $?")
                    if (response.trim { it <= ' ' } == "0") {
                        restartController()
                        Timber.d("wifi country successfully set, rebooting artoo")
                        txPowerCompliantCountry.set(compliantCountry)
                        postSuccessEvent(listener)
                    } else {
                        Timber.d("wifi country set failed: %s", response)
                        postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                    }
                }
            } catch (e: IOException) {
                Timber.e(e, "Error occurred while changing wifi country.")
                postTimeoutEvent(listener)
            }
        }
    }

    private fun loadCurrentEUTxPowerComplianceMode() {
        postAsyncTask(checkEUTxPowerCompliance)
    }

    private fun loadCurrentControllerMode() {
        postAsyncTask(artooModeRetriever)
    }

    private fun loadControllerUnit() {
        postAsyncTask(unitsRetriever)
    }

    private fun restartController() {
        try {
            Companion.sshLink.execute(SOLOLINK_SSID_CONFIG_PATH + " --reboot")
        } catch (e: IOException) {
            Timber.e(e, "Error occurred while restarting hostpad service on Artoo.")
        }
    }

    /**
     * Refresh the vehicle's components versions
     */
    fun refreshControllerVersions() {
        updateArtooVersion()
        updateStm32Version()
    }

    companion object {
        /**
         * This is the minimum version that provides an api for the controller mode update.
         */
        private val CONTROLLER_MODE_MIN_VERSION = Version.forIntegers(1, 1, 13)
        const val SOLOLINK_SSID_CONFIG_PATH = "/usr/bin/sololink_config"
        private const val ARTOO_VERSION_FILENAME = "/VERSION"
        private const val STM32_VERSION_FILENAME = "/STM_VERSION"

        /**
         * Artoo link ip address
         */
        const val ARTOO_IP = "10.1.1.1"
        private const val ARTOO_VIDEO_HANDSHAKE_PORT = 5502
        private const val ARTOO_BUTTON_PORT = 5016
        private const val ARTOO_BATTERY_PORT = 5021
        const val ARTOO_UDP_PORT = 5600
        protected val sshLink = SshConnection(ARTOO_IP, SoloComp.SSH_USERNAME,
                SoloComp.SSH_PASSWORD)
    }

    init {
        videoHandshake.setIpConnectionListener(object : IpConnectionListener {
            override fun onIpConnected() {
                handler.removeCallbacks(reconnectVideoHandshake)
                Timber.d("Artoo link connected. Starting video stream...")
                streamingPermission.set(true)
            }

            override fun onIpDisconnected() {
                streamingPermission.set(false)
                if (isVideoHandshakeStarted.get()) handler.postDelayed(reconnectVideoHandshake, RECONNECT_COUNTDOWN)
            }

            override fun onPacketReceived(packetBuffer: ByteBuffer) {}
        })

        batteryConnection.setIpConnectionListener(object : IpConnectionListener {
            override fun onIpConnected() {
                handler.removeCallbacks(reconnectBatteryTask)
            }

            override fun onIpDisconnected() {
                //Try to connect
                if (isBatteryStarted.get()) {
                    handler.postDelayed(reconnectBatteryTask, RECONNECT_COUNTDOWN)
                }
            }

            override fun onPacketReceived(packetBuffer: ByteBuffer) {
                val tlvMsgs = TLVMessageParser.parseTLVPacket(packetBuffer)
                if (tlvMsgs.isEmpty()) return
                for (tlvMsg in tlvMsgs) {
                    val messageType = tlvMsg.messageType
                    Timber.d("Received tlv message: $messageType")
                    if (linkListener != null) linkListener!!.onTlvPacketReceived(tlvMsg)
                }
            }
        })
    }
}
