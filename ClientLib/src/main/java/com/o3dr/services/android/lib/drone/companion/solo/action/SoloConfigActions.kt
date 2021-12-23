package com.o3dr.services.android.lib.drone.companion.solo.action

import com.o3dr.services.android.lib.drone.companion.solo.action.SoloConfigActions

/**
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
object SoloConfigActions {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.companion.solo.action.config"
    const val ACTION_UPDATE_WIFI_SETTINGS = "$PACKAGE_NAME.UPDATE_WIFI_SETTINGS"
    const val EXTRA_WIFI_SSID = "extra_wifi_ssid"
    const val EXTRA_WIFI_PASSWORD = "extra_wifi_password"
    const val ACTION_UPDATE_BUTTON_SETTINGS = "$PACKAGE_NAME.UPDATE_BUTTON_SETTINGS"

    /**
     * Used to retrieve the button settings to push to the sololink companion computer.
     *
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSettingSetter}
     */
    const val EXTRA_BUTTON_SETTINGS = "extra_button_settings"
    const val ACTION_UPDATE_CONTROLLER_MODE = "$PACKAGE_NAME.UPDATE_CONTROLLER_MODE"

    /**
     * Controller mode to apply.
     *
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode}
     */
    const val EXTRA_CONTROLLER_MODE = "extra_controller_mode"
    const val ACTION_UPDATE_TX_POWER_COMPLIANCE_COUNTRY = "$PACKAGE_NAME.UPDATE_TX_POWER_COMPLIANCE_COUNTRY"

    /**
     * String value. The country the controller should be made compliant with.
     */
    const val EXTRA_TX_POWER_COMPLIANT_COUNTRY_CODE = "extra_tx_power_compliant_country_code"
    const val ACTION_REFRESH_SOLO_VERSIONS = "$PACKAGE_NAME.REFRESH_SOLO_VERSIONS"
    const val ACTION_UPDATE_CONTROLLER_UNIT = "$PACKAGE_NAME.UPDATE_CONTROLLER_UNIT"

    /**
     * Controller unit system to apply.
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit}
     */
    const val EXTRA_CONTROLLER_UNIT = "extra_controller_unit"
}
