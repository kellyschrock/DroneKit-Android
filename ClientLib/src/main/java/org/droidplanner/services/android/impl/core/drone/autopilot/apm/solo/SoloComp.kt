package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.RemoteException
import android.text.TextUtils
import android.util.SparseArray
import com.o3dr.android.client.BuildConfig
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.companion.solo.SoloEventExtras
import com.o3dr.services.android.lib.drone.companion.solo.SoloEvents
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonPacket
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonTypes
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.tlv.*
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.controller.ControllerLinkListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.controller.ControllerLinkManager
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.sololink.SoloLinkListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.sololink.SoloLinkManager
import org.droidplanner.services.android.impl.utils.NetworkUtils
import timber.log.Timber
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

/**
 * Sololink companion computer implementation
 */
class SoloComp(
        private val context: Context,
        private val soloIp: String,
        private val handler: Handler?)
: SoloLinkListener, ControllerLinkListener {

    interface SoloCompListener {
        fun onConnected()
        fun onDisconnected()
        fun onTlvPacketReceived(packet: TLVPacket?)
        fun onPresetButtonLoaded(buttonType: Int, buttonSettings: SoloButtonSetting?)
        fun onWifiInfoUpdated(wifiName: String?, wifiPassword: String?)
        fun onButtonPacketReceived(packet: ButtonPacket?)
        fun onTxPowerComplianceCountryUpdated(compliantCountry: String?)
        fun onVersionsUpdated()
        fun onControllerEvent(event: String?, eventInfo: Bundle?)
    }

    private val asyncExecutor: ExecutorService? = Executors.newCachedThreadPool()
    private val controllerLinkManager: ControllerLinkManager = ControllerLinkManager(context, handler!!, asyncExecutor)
    private val soloLinkMgr: SoloLinkManager = SoloLinkManager(context, soloIp, handler, asyncExecutor)
    private var compListener: SoloCompListener? = null

    var goproState: SoloGoproState? = null
        private set

    var goproStateV2: SoloGoproStateV2? = null
        private set

    val wifiSettings: android.util.Pair<String, String>
        get() = controllerLinkManager.wifiSettings

    fun hasStreamingPermission(): Boolean {
        return if (BuildConfig.SITL_DEBUG) true else controllerLinkManager.hasStreamingPermission()
    }

    fun setListener(listener: SoloCompListener?) {
        compListener = listener
    }

    fun start() {
        if (!isAvailable(context)) {
            return
        }

        if (!BuildConfig.SITL_DEBUG) {
            controllerLinkManager.start(this)
        }

        soloLinkMgr.start(this)
    }

    fun stop() {
        soloLinkMgr.stop()
        if (!BuildConfig.SITL_DEBUG) {
            controllerLinkManager.stop()
        }
    }

    fun refreshState() {
        soloLinkMgr.refreshState()
        if (!BuildConfig.SITL_DEBUG) controllerLinkManager.refreshState()
    }

    /**
     * Terminates and releases resources used by this companion computer instance. The instance should no longer be used after calling this method.
     */
    fun destroy() {
        stop()
        asyncExecutor!!.shutdownNow()
    }

    override fun onTlvPacketReceived(packet: TLVPacket?) {
        if (packet == null) return
        when (packet.messageType) {
            TLVMessageTypes.TYPE_SOLO_GOPRO_STATE -> {
                goproState = packet as SoloGoproState?
                Timber.d("Updated gopro state.")
            }
            TLVMessageTypes.TYPE_SOLO_GOPRO_STATE_V2 -> {
                goproStateV2 = packet as SoloGoproStateV2?
                Timber.i("Updated gopro state.")
            }
        }
        if (compListener != null) compListener!!.onTlvPacketReceived(packet)
    }

    override fun onWifiInfoUpdated(wifiName: String, wifiPassword: String) {
        if (compListener != null) compListener!!.onWifiInfoUpdated(wifiName, wifiPassword)
    }

    override fun onButtonPacketReceived(packet: ButtonPacket) {
        if (compListener != null) compListener!!.onButtonPacketReceived(packet)
    }

    override fun onTxPowerComplianceCountryUpdated(compliantCountry: String) {
        if (compListener != null) compListener!!.onTxPowerComplianceCountryUpdated(compliantCountry)
    }

    override fun onControllerModeUpdated() {
        if (compListener != null) {
            val eventInfo = Bundle()
            eventInfo.putInt(SoloEventExtras.EXTRA_SOLO_CONTROLLER_MODE, controllerMode)
            compListener!!.onControllerEvent(SoloEvents.SOLO_CONTROLLER_MODE_UPDATED, eventInfo)
        }
    }

    override fun onControllerUnitUpdated(trimmedResponse: String) {
        if (compListener != null) {
            val eventInfo = Bundle()
            eventInfo.putString(SoloEventExtras.EXTRA_SOLO_CONTROLLER_UNIT, trimmedResponse)
            compListener!!.onControllerEvent(SoloEvents.SOLO_CONTROLLER_UNIT_UPDATED, eventInfo)
        }
    }

    override fun onPresetButtonLoaded(buttonType: Int, buttonSettings: SoloButtonSetting?) {
        if (compListener != null) compListener!!.onPresetButtonLoaded(buttonType, buttonSettings)
    }

    override fun onLinkConnected() {
        if (isConnected) {
            if (compListener != null) compListener!!.onConnected()
        } else {
            if (!controllerLinkManager.isLinkConnected && !BuildConfig.SITL_DEBUG) controllerLinkManager.start(this)
            if (!soloLinkMgr.isLinkConnected) soloLinkMgr.start(this)
        }
    }

    override fun onLinkDisconnected() {
        if (compListener != null) compListener!!.onDisconnected()
        soloLinkMgr.stop()
        if (!BuildConfig.SITL_DEBUG) {
            controllerLinkManager.stop()
        }
    }

    override fun onVersionsUpdated() {
        if (compListener != null) compListener!!.onVersionsUpdated()
    }

    override fun onMacAddressUpdated() {
        val soloMacAddress = soloLinkMgr.macAddress
        val artooMacAddress = controllerLinkManager.macAddress
        if (!TextUtils.isEmpty(soloMacAddress) && !TextUtils.isEmpty(artooMacAddress) && compListener != null) {
            compListener!!.onControllerEvent(AttributeEvent.STATE_VEHICLE_UID, null)
        }
    }

    val isConnected: Boolean
        get() = if (BuildConfig.SITL_DEBUG) soloLinkMgr.isLinkConnected else controllerLinkManager.isLinkConnected && soloLinkMgr.isLinkConnected
    val txPowerCompliantCountry: String
        get() = controllerLinkManager.getTxPowerCompliantCountry()

    fun refreshSoloVersions() {
        soloLinkMgr.refreshSoloLinkVersions()
        if (!BuildConfig.SITL_DEBUG) controllerLinkManager.refreshControllerVersions()
    }

    val controllerFirmwareVersion: String
        get() = controllerLinkManager.getStm32Version()

    val vehicleVersion: String
        get() = soloLinkMgr.getVehicleVersion()

    @get:ControllerMode
    val controllerMode: Int
        get() = controllerLinkManager.getControllerMode()

    @get:ControllerUnit
    val controllerUnit: String
        get() = controllerLinkManager.controllerUnit
    val soloMacAddress: String
        get() = soloLinkMgr.macAddress
    val controllerMacAddress: String
        get() = controllerLinkManager.macAddress
    val autopilotVersion: String
        get() = soloLinkMgr.getPixhawkVersion()
    val gimbalVersion: String
        get() = soloLinkMgr.getGimbalVersion()

    fun getButtonSetting(buttonType: Int): SoloButtonSetting? {
        return soloLinkMgr.getLoadedPresetButton(buttonType)
    }

    val buttonSettings: SparseArray<SoloButtonSetting?>
        get() {
            val buttonSettings = SparseArray<SoloButtonSetting?>(2)
            buttonSettings.append(ButtonTypes.BUTTON_A, soloLinkMgr.getLoadedPresetButton(ButtonTypes.BUTTON_A))
            buttonSettings.append(ButtonTypes.BUTTON_B, soloLinkMgr.getLoadedPresetButton(ButtonTypes.BUTTON_B))
            return buttonSettings
        }

    fun sendSoloLinkMessage(message: TLVPacket?, listener: ICommandListener?) {
        soloLinkMgr.sendTLVPacket(message, listener)
    }

    fun updateWifiSettings(wifiSsid: String?, wifiPassword: String?,
                           listener: ICommandListener?) {
        postAsyncTask {
            if (soloLinkMgr.updateSololinkWifi(wifiSsid!!, wifiPassword!!)
                    && controllerLinkManager.updateSololinkWifi(wifiSsid, wifiPassword)) {
                Timber.d("Sololink wifi update successful.")
                listener?.let { postSuccessEvent(it) }
            } else {
                Timber.d("Sololink wifi update failed.")
                if (listener != null) {
                    postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                }
            }
        }
    }

    fun pushButtonSettings(buttonSettings: SoloButtonSettingSetter?, listener: ICommandListener?) {
        soloLinkMgr.pushPresetButtonSettings(buttonSettings, listener)
    }

    fun updateControllerMode(@ControllerMode selectedMode: Int, listener: ICommandListener?) {
        controllerLinkManager.updateControllerMode(selectedMode, listener)
    }

    fun updateControllerUnit(@ControllerUnit selectedUnit: String?, listener: ICommandListener?) {
        controllerLinkManager.updateControllerUnit(selectedUnit!!, listener)
    }

    fun updateTxPowerComplianceCountry(compliantCountry: String?, listener: ICommandListener?) {
        controllerLinkManager.setTxPowerComplianceCountry(compliantCountry!!, listener)
    }

    protected fun postAsyncTask(task: Runnable?) {
        if (asyncExecutor != null && !asyncExecutor.isShutdown) {
            asyncExecutor.execute(task)
        }
    }

    protected fun postSuccessEvent(listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onSuccess()
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
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
                    Timber.e(e, e.message)
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
                    Timber.e(e, e.message)
                }
            })
        }
    }

    fun enableFollowDataConnection() {
        soloLinkMgr.enableFollowDataConnection()
    }

    fun disableFollowDataConnection() {
        soloLinkMgr.disableFollowDataConnection()
    }

    fun updateFollowCenter(location: SoloMessageLocation?) {
        soloLinkMgr.sendTLVPacket(location, true, null)
    }

    companion object {
        const val SOLO_LINK_WIFI_PREFIX = "SoloLink_"
        const val SSH_USERNAME = "root"
        const val SSH_PASSWORD = "TjSDBkAu"
        @JvmStatic
        fun isAvailable(context: Context?): Boolean {
            return NetworkUtils.isOnSololinkNetwork(context)
        }
    }
}
