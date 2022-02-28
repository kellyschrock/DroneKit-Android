package org.droidplanner.services.android.impl.utils

import android.os.RemoteException
import android.text.TextUtils
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.companion.solo.SoloState
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSettingSetter
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.ArduSolo
import timber.log.Timber

/**
 * Created by Fredia Huya-Kouadio on 7/29/15.
 */
object SoloApiUtils {
    fun getSoloLinkState(arduSolo: ArduSolo?): SoloState? {
        if (arduSolo == null) return null
        val soloComp = arduSolo.soloComp
        val wifiSettings = soloComp.wifiSettings

        return SoloState(soloComp.autopilotVersion, soloComp.controllerFirmwareVersion,
                soloComp.controllerFirmwareVersion, soloComp.vehicleVersion,
                wifiSettings.second, wifiSettings.first, soloComp.txPowerCompliantCountry,
                soloComp.buttonSettings, soloComp.gimbalVersion,
                soloComp.controllerMode, soloComp.controllerUnit)
    }

    fun isSoloLinkFeatureAvailable(drone: Drone?, listener: ICommandListener?): Boolean {
        if (drone == null) return false
        if (drone !is ArduSolo) {
            if (listener != null) {
                try {
                    listener.onError(CommandExecutionError.COMMAND_UNSUPPORTED)
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            }
            return false
        }
        return true
    }

    fun sendSoloLinkMessage(arduSolo: ArduSolo, messageData: TLVPacket?,
                            listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener) || messageData == null) return
        val soloComp = arduSolo.soloComp
        soloComp.sendSoloLinkMessage(messageData, listener)
    }

    fun updateSoloLinkWifiSettings(arduSolo: ArduSolo,
                                   wifiSsid: String?, wifiPassword: String?,
                                   listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener)) return
        if (TextUtils.isEmpty(wifiSsid) && TextUtils.isEmpty(wifiPassword)) return
        val soloComp = arduSolo.soloComp
        soloComp.updateWifiSettings(wifiSsid, wifiPassword, listener)
    }

    fun updateSoloLinkButtonSettings(arduSolo: ArduSolo,
                                     buttonSettings: SoloButtonSettingSetter?,
                                     listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener) || buttonSettings == null) return
        val soloComp = arduSolo.soloComp
        soloComp.pushButtonSettings(buttonSettings, listener)
    }

    fun updateSoloLinkControllerMode(arduSolo: ArduSolo,
                                     @ControllerMode mode: Int,
                                     listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener)) return
        val soloComp = arduSolo.soloComp
        soloComp.updateControllerMode(mode, listener)
    }

    fun updateSoloControllerUnit(arduSolo: ArduSolo, @ControllerUnit unit: String?, listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener)) return
        val soloComp = arduSolo.soloComp
        soloComp.updateControllerUnit(unit, listener)
    }

    fun updateSoloLinkTxPowerComplianceCountry(arduSolo: ArduSolo, compliantCountry: String?, listener: ICommandListener?) {
        if (!isSoloLinkFeatureAvailable(arduSolo, listener)) return
        val soloComp = arduSolo.soloComp
        soloComp.updateTxPowerComplianceCountry(compliantCountry, listener)
    }
}
