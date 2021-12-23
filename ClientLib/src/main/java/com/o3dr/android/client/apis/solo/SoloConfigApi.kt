package com.o3dr.android.client.apis.solo

import com.o3dr.android.client.apis.solo.SoloApi
import com.o3dr.services.android.lib.model.AbstractCommandListener
import android.os.Bundle
import com.MAVLink.ardupilotmega.msg_led_control
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.drone.companion.solo.action.SoloConfigActions
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSettingSetter
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.android.client.utils.TxPowerComplianceCountries
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.action.ExperimentalActions
import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.android.client.apis.solo.SoloConfigApi
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
class SoloConfigApi protected constructor(drone: Drone?) : SoloApi(drone!!) {
    /**
     * Updates the wifi settings for the solo vehicle.
     *
     * @param wifiSsid     Updated wifi ssid
     * @param wifiPassword Updated wifi password
     * @param listener     Register a callback to receive update of the command execution status.
     */
    fun updateWifiSettings(wifiSsid: String?, wifiPassword: String?, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putString(SoloConfigActions.EXTRA_WIFI_SSID, wifiSsid)
            putString(SoloConfigActions.EXTRA_WIFI_PASSWORD, wifiPassword)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_UPDATE_WIFI_SETTINGS, params), listener)
    }

    /**
     * Updates the button settings for the solo vehicle.
     *
     * @param buttonSettings Updated button settings.
     * @param listener       Register a callback to receive update of the command execution status.
     */
    fun updateButtonSettings(buttonSettings: SoloButtonSettingSetter?, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putParcelable(SoloConfigActions.EXTRA_BUTTON_SETTINGS, buttonSettings)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_UPDATE_BUTTON_SETTINGS, params), listener)
    }

    /**
     * Updates the controller mode (joystick mapping)
     *
     * @param controllerMode Controller mode. @see [com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode]
     * @param listener       Register a callback to receive update of the command execution status.
     */
    fun updateControllerMode(@ControllerMode controllerMode: Int, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putInt(SoloConfigActions.EXTRA_CONTROLLER_MODE, controllerMode)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_UPDATE_CONTROLLER_MODE, params), listener)
    }

    /**
     * Updates the tx power compliance to the specified country.
     *
     * @param compliantCountry Country code which the controller will be compliant.
     * @param listener    Register a callback to receive update of the command execution status.
     */
    fun updateTxPowerComplianceCountry(compliantCountry: TxPowerComplianceCountries, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putString(SoloConfigActions.EXTRA_TX_POWER_COMPLIANT_COUNTRY_CODE, compliantCountry.name)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_UPDATE_TX_POWER_COMPLIANCE_COUNTRY, params), listener)
    }

    /**
     * Refresh the solo versions info.
     */
    fun refreshSoloVersions() {
        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_REFRESH_SOLO_VERSIONS), null)
    }

    /**
     * Updates the controller unit system.
     * @param controllerUnit Controller unit system. @see [com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit]
     * @param listener Register a callback that receive update of the command execution status.
     */
    fun updateControllerUnit(@ControllerUnit controllerUnit: String, listener: AbstractCommandListener?) {
        if (SoloControllerUnits.UNKNOWN == controllerUnit) {
            if (listener != null) {
                drone.post { listener.onError(CommandExecutionError.COMMAND_DENIED) }
            }
            return
        }

        val params = Bundle().apply {
            putString(SoloConfigActions.EXTRA_CONTROLLER_UNIT, controllerUnit)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloConfigActions.ACTION_UPDATE_CONTROLLER_UNIT, params), listener)
    }

    fun setLedColor(instance: Int, pattern: Int) {
        val led = msg_led_control().apply {
            this.instance = instance.toShort()
            this.pattern = pattern.toShort() // actually, macro
        }

        val params = Bundle().apply {
            putParcelable(ExperimentalActions.EXTRA_MAVLINK_MESSAGE, MavlinkMessageWrapper(led))
        }

        drone.performAsyncAction(Action(ExperimentalActions.ACTION_SEND_MAVLINK_MESSAGE, params))
    }

    companion object {
        private val soloConfigApiCache = ConcurrentHashMap<Drone, SoloConfigApi>()
        private val apiBuilder: Builder<SoloConfigApi> = Builder { drone -> SoloConfigApi(drone) }

        /**
         * Retrieves a sololink api instance.
         *
         * @param drone target vehicle
         * @return a SoloLinkApi instance.
         */
        fun getApi(drone: Drone?): SoloConfigApi {
            return getApi(drone, soloConfigApiCache, apiBuilder)
        }
    }
}
