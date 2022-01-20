package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.RemoteException
import android.text.TextUtils
import android.view.Surface
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_statustext
import com.MAVLink.enums.MAV_TYPE
import com.o3dr.android.client.apis.CapabilityApi
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.companion.solo.SoloAttributes
import com.o3dr.services.android.lib.drone.companion.solo.SoloEventExtras
import com.o3dr.services.android.lib.drone.companion.solo.SoloEvents
import com.o3dr.services.android.lib.drone.companion.solo.action.SoloActions
import com.o3dr.services.android.lib.drone.companion.solo.action.SoloConfigActions
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonPacket
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSetting
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSettingSetter
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVMessageTypes
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.drone.property.State
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduCopter
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp.SoloCompListener
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import org.droidplanner.services.android.impl.core.drone.variables.HeartBeat
import org.droidplanner.services.android.impl.core.drone.variables.StreamRates
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import org.droidplanner.services.android.impl.utils.SoloApiUtils
import timber.log.Timber
import java.util.regex.Pattern

class ArduSolo(
        droneId: String?,
        soloIp: String?,
        context: Context?,
        mavClient: DataLinkProvider<MAVLinkMessage?>?,
        handler: Handler,
       warningParser: AutopilotWarningParser?,
        logListener: LogMessageListener?)
: ArduCopter(droneId, context, mavClient, handler, warningParser, logListener) {

    private val disconnectSoloCompTask: Runnable = object : Runnable {
        override fun run() {
            if (soloComp != null && soloComp.isConnected) {
                soloComp.stop()
            }
            handler.removeCallbacks(this)
        }
    }

    private var pixhawkSerialNumber: String? = null
    val soloComp: SoloComp = SoloComp(context!!, soloIp!!, handler)

    override fun destroy() {
        super.destroy()
        soloComp!!.destroy()
    }

    /**
     * No need to update the stream rates for Solo as it's being set by the companion computer
     * @return
     */
    override val streamRates: StreamRates?
        get() = null

    override val type: Int
        get() = MAV_TYPE.MAV_TYPE_QUADROTOR

    public override fun setType(type: Int) {}
    override val firmwareType: FirmwareType
        get() = FirmwareType.ARDU_SOLO

    override val isConnected: Boolean
        get() = soloComp!!.isConnected && super.isConnected

    override fun getAttribute(attributeType: String?): DroneAttribute? {
        return when (attributeType) {
            SoloAttributes.SOLO_STATE -> SoloApiUtils.getSoloLinkState(this)
            SoloAttributes.SOLO_GOPRO_STATE -> soloComp!!.goproState
            SoloAttributes.SOLO_GOPRO_STATE_V2 -> soloComp!!.goproStateV2
            AttributeType.STATE -> {
                val stateAttr = super.getAttribute(attributeType) as State?
                stateAttr!!.addToVehicleUid(SERIAL_NUMBER_LABEL, pixhawkSerialNumber)
                stateAttr.addToVehicleUid("solo_mac_address", soloComp!!.soloMacAddress)
                stateAttr.addToVehicleUid("controller_mac_address", soloComp.controllerMacAddress)
                stateAttr
            }
            else -> super.getAttribute(attributeType)
        }
    }

    protected fun resetVideoManager() {
        videoMgr.reset()
    }

    override fun startVideoStream(videoProps: Bundle, appId: String, newVideoTag: String, videoSurface: Surface,
                                  listener: ICommandListener?) {
        if (!soloComp!!.hasStreamingPermission()) {
            postErrorEvent(CommandExecutionError.COMMAND_DENIED, listener)
            return
        }
        super.startVideoStream(videoProps, appId, newVideoTag, videoSurface, listener)
    }

    protected fun postErrorEvent(error: Int, listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post {
                try {
                    listener.onError(error)
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            }
        }
    }

    override fun notifyDroneEvent(event: DroneEventsType?) {
        when (event) {
            DroneEventsType.HEARTBEAT_FIRST, DroneEventsType.CONNECTED -> {
                Timber.i("Vehicle " + event.name.toLowerCase())
                //Try connecting the companion computer
                if (!soloComp!!.isConnected) {
                    resetVideoManager()
                    soloComp.start()
                    return
                }
            }
            DroneEventsType.DISCONNECTED -> {
                Timber.i("Vehicle disconnected.")
                if (soloComp!!.isConnected) {
                    soloComp.stop()
                    resetVideoManager()
                    return
                }
            }
            DroneEventsType.HEARTBEAT_TIMEOUT -> {
                Timber.i("Vehicle heartbeat timed out.")
                if (soloComp!!.isConnected) {
                    //Start a countdown at the conclusion of which, disconnect the solo companion computer.
                    handler.postDelayed(disconnectSoloCompTask, HeartBeat.HEARTBEAT_NORMAL_TIMEOUT)
                }
            }
            DroneEventsType.HEARTBEAT_RESTORED -> {
                Timber.i("Vehicle heartbeat restored.")
                //Dismiss the countdown to disconnect the solo companion computer.
                handler.removeCallbacks(disconnectSoloCompTask)
                if (!soloComp!!.isConnected) {
                    soloComp.start()
                } else {
                    soloComp.refreshState()
                }
            }
        }
        super.notifyDroneEvent(event)
    }

    override fun executeAsyncAction(action: Action?, listener: ICommandListener?): Boolean {
        val type = action!!.type
        val data = action.data
        return when (type) {
            SoloActions.ACTION_SEND_MESSAGE -> {
                val messageData: TLVPacket = data!!.getParcelable(SoloActions.EXTRA_MESSAGE_DATA)
                if (messageData != null) {
                    SoloApiUtils.sendSoloLinkMessage(this, messageData, listener)
                }
                true
            }
            SoloConfigActions.ACTION_UPDATE_WIFI_SETTINGS -> {
                val wifiSsid = data!!.getString(SoloConfigActions.EXTRA_WIFI_SSID)
                val wifiPassword = data.getString(SoloConfigActions.EXTRA_WIFI_PASSWORD)
                SoloApiUtils.updateSoloLinkWifiSettings(this, wifiSsid, wifiPassword, listener)
                true
            }
            SoloConfigActions.ACTION_UPDATE_BUTTON_SETTINGS -> {
                val buttonSettings: SoloButtonSettingSetter = data!!.getParcelable(SoloConfigActions.EXTRA_BUTTON_SETTINGS)
                if (buttonSettings != null) {
                    SoloApiUtils.updateSoloLinkButtonSettings(this, buttonSettings, listener)
                }
                true
            }
            SoloConfigActions.ACTION_UPDATE_CONTROLLER_MODE -> {
                @ControllerMode val mode = data!!.getInt(SoloConfigActions.EXTRA_CONTROLLER_MODE)
                SoloApiUtils.updateSoloLinkControllerMode(this, mode, listener)
                true
            }
            SoloConfigActions.ACTION_UPDATE_TX_POWER_COMPLIANCE_COUNTRY -> {
                val compliantCountry = data!!.getString(SoloConfigActions.EXTRA_TX_POWER_COMPLIANT_COUNTRY_CODE)
                SoloApiUtils.updateSoloLinkTxPowerComplianceCountry(this, compliantCountry, listener)
                true
            }
            SoloConfigActions.ACTION_REFRESH_SOLO_VERSIONS -> {
                soloComp!!.refreshSoloVersions()
                true
            }
            SoloConfigActions.ACTION_UPDATE_CONTROLLER_UNIT -> {
                @ControllerUnit val unit = data!!.getString(SoloConfigActions.EXTRA_CONTROLLER_UNIT)
                SoloApiUtils.updateSoloControllerUnit(this, unit, listener)
                true
            }
            else -> super.executeAsyncAction(action, listener)
        }
    }

    override fun isFeatureSupported(featureId: String): Boolean {
        return when (featureId) {
            CapabilityApi.FeatureIds.SOLO_VIDEO_STREAMING, CapabilityApi.FeatureIds.COMPASS_CALIBRATION, CapabilityApi.FeatureIds.KILL_SWITCH -> true
            else -> super.isFeatureSupported(featureId)
        }
    }

    override fun processSignalUpdate(rxerrors: Int, fixed: Int, rssi: Short, remrssi: Short, txbuf: Short,
                                     noise: Short, remnoise: Short) {
        val unsignedRemRssi: Double = (remrssi.toInt() and 0xFF).toDouble()
        signal.isValid = true
        signal.rxerrors = rxerrors and 0xFFFF
        signal.fixed = fixed and 0xFFFF
        signal.rssi = (rssi.toInt() and 0xFF).toDouble()
        signal.remrssi = unsignedRemRssi
        signal.noise = (noise.toInt() and 0xFF).toDouble()
        signal.remnoise = (remnoise.toInt() and 0xFF).toDouble()
        signal.txbuf = txbuf.toInt() and 0xFF
        val signalStrength = if (unsignedRemRssi <= 127) unsignedRemRssi else unsignedRemRssi - 256
        signal.signalStrength = signalStrength
        notifyDroneEvent(DroneEventsType.RADIO)
    }

    override fun processStatusText(statusText: msg_statustext?) {
        super.processStatusText(statusText)
        val message = statusText!!.getText()
        if (!TextUtils.isEmpty(message)) {

            //Parse pixhawk serial number.
            val matcher = PIXHAWK_SERIAL_NUMBER_PATTERN.matcher(message)
            if (matcher.matches()) {
                Timber.i("Received serial number: %s", message)
                val serialNumber = matcher.group(2) + matcher.group(3) + matcher.group(4)
                if (!serialNumber.equals(pixhawkSerialNumber, ignoreCase = true)) {
                    pixhawkSerialNumber = serialNumber
                    notifyAttributeListener(AttributeEvent.STATE_VEHICLE_UID)
                }
            }
        }
    }

    override fun brakeVehicle(listener: ICommandListener?): Boolean {
        state?.changeFlightMode(ApmModes.ROTOR_BRAKE, listener)
        return true
    }

    companion object {
        private const val PIXHAWK_SERIAL_NUMBER_REGEX = ".*PX4v2 (([0-9A-F]{8}) ([0-9A-F]{8}) ([0-9A-F]{8}))"
        private val PIXHAWK_SERIAL_NUMBER_PATTERN = Pattern.compile(PIXHAWK_SERIAL_NUMBER_REGEX)
        private const val SERIAL_NUMBER_LABEL = "serial_number"
    }

    init {
        soloComp.setListener(object : SoloCompListener {
            override fun onConnected() {
                if (isConnected) {
                    notifyDroneEvent(DroneEventsType.CONNECTED)
                }
            }

            override fun onDisconnected() {
                notifyDroneEvent(DroneEventsType.DISCONNECTED)
            }

            override fun onTlvPacketReceived(packet: TLVPacket?) {
                when (packet!!.messageType) {
                    TLVMessageTypes.TYPE_ARTOO_INPUT_REPORT_MESSAGE -> {}
                    TLVMessageTypes.TYPE_SOLO_GET_BUTTON_SETTING, TLVMessageTypes.TYPE_SOLO_SET_BUTTON_SETTING -> {}
                    TLVMessageTypes.TYPE_SOLO_GOPRO_STATE -> notifyAttributeListener(SoloEvents.SOLO_GOPRO_STATE_UPDATED)
                    TLVMessageTypes.TYPE_SOLO_GOPRO_STATE_V2 -> notifyAttributeListener(SoloEvents.SOLO_GOPRO_STATE_V2_UPDATED)
                    else -> {
                        val messageInfo = Bundle()
                        messageInfo.putParcelable(SoloEventExtras.EXTRA_SOLO_MESSAGE_DATA, packet)
                        notifyAttributeListener(SoloEvents.SOLO_MESSAGE_RECEIVED, messageInfo)
                    }
                }
            }

            override fun onPresetButtonLoaded(buttonType: Int, buttonSettings: SoloButtonSetting?) {
                notifyAttributeListener(SoloEvents.SOLO_BUTTON_SETTINGS_UPDATED, null)
            }

            override fun onWifiInfoUpdated(wifiName: String?, wifiPassword: String?) {
                notifyAttributeListener(SoloEvents.SOLO_WIFI_SETTINGS_UPDATED, null)
            }

            override fun onButtonPacketReceived(packet: ButtonPacket?) {
                val eventInfo = Bundle()
                eventInfo.putParcelable(SoloEventExtras.EXTRA_SOLO_BUTTON_EVENT, packet)
                notifyAttributeListener(SoloEvents.SOLO_BUTTON_EVENT_RECEIVED, eventInfo)
            }

            override fun onTxPowerComplianceCountryUpdated(compliantCountry: String?) {
                val eventInfo = Bundle(1)
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_TX_POWER_COMPLIANT_COUNTRY, compliantCountry)
                notifyAttributeListener(SoloEvents.SOLO_TX_POWER_COMPLIANCE_COUNTRY_UPDATED, eventInfo)
            }

            override fun onVersionsUpdated() {
                val eventInfo = Bundle()
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_VEHICLE_VERSION, soloComp.vehicleVersion)
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_AUTOPILOT_VERSION, soloComp.autopilotVersion)
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_GIMBAL_VERSION, soloComp.gimbalVersion)
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_CONTROLLER_VERSION, soloComp.controllerFirmwareVersion)
                eventInfo.putString(SoloEventExtras.EXTRA_SOLO_CONTROLLER_FIRMWARE_VERSION, soloComp.controllerFirmwareVersion)
                notifyAttributeListener(SoloEvents.SOLO_VERSIONS_UPDATED, eventInfo)
            }

            override fun onControllerEvent(event: String?, eventInfo: Bundle?) {
                notifyAttributeListener(event, eventInfo)
            }
        })
    }
}
