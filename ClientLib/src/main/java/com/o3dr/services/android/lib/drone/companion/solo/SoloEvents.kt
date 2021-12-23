package com.o3dr.services.android.lib.drone.companion.solo

import com.o3dr.services.android.lib.drone.companion.solo.SoloEvents

/**
 * Stores all possible drone events.
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
object SoloEvents {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.companion.solo.event"

    /**
     * Broadcasts updates to the GoPro state.
     *
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproState}
     */
    const val SOLO_GOPRO_STATE_UPDATED = PACKAGE_NAME + ".GOPRO_STATE_UPDATED"

    /**
     * Broadcasts updates to the Gopro extended state.
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloGoproStateV2}
     */
    const val SOLO_GOPRO_STATE_V2_UPDATED = PACKAGE_NAME + ".GOPRO_STATE_V2_UPDATED"

    /**
     * Signals update to the sololink wifi settings
     *
     * @see {@link SoloState}
     */
    const val SOLO_WIFI_SETTINGS_UPDATED = PACKAGE_NAME + ".SOLO_WIFI_SETTINGS_UPDATED"

    /**
     * Signals update to the sololink button settings
     *
     * @see {@link SoloState}
     */
    const val SOLO_BUTTON_SETTINGS_UPDATED = PACKAGE_NAME + ".SOLO_BUTTON_SETTINGS_UPDATED"

    /**
     * Triggers every time a button event occurs.
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_BUTTON_EVENT}
     */
    const val SOLO_BUTTON_EVENT_RECEIVED = PACKAGE_NAME + ".SOLO_BUTTON_EVENT_RECEIVED"

    /**
     * Triggers upon receipt of a sololink message.
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_MESSAGE_DATA}
     */
    const val SOLO_MESSAGE_RECEIVED = PACKAGE_NAME + ".SOLO_MESSAGE_RECEIVED"

    /**
     * Triggers upon updates to the tx power compliance.
     */
    const val SOLO_TX_POWER_COMPLIANCE_COUNTRY_UPDATED = PACKAGE_NAME + ".SOLO_TX_POWER_COMPLIANCE_COUNTRY_UPDATED"

    /**
     * Triggers upon updates of the solo versions
     * @see {@link SoloEventExtras.EXTRA_SOLO_CONTROLLER_VERSION}
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_CONTROLLER_FIRMWARE_VERSION}
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_AUTOPILOT_VERSION}
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_VEHICLE_VERSION}
     *
     * @see {@link SoloEventExtras.EXTRA_SOLO_GIMBAL_VERSION}
     */
    const val SOLO_VERSIONS_UPDATED = PACKAGE_NAME + ".SOLO_VERSIONS_UPDATED"

    /**
     * Triggers upon updates to the controller mode
     * @see {@link SoloEventExtras.EXTRA_SOLO_CONTROLLER_MODE}
     */
    const val SOLO_CONTROLLER_MODE_UPDATED = PACKAGE_NAME + ".SOLO_CONTROLLER_MODE_UPDATED"

    /**
     * Triggers upon updates to the controller unit system
     * @see {@link SoloEventExtras.EXTRA_SOLO_CONTROLLER_UNIT}
     */
    const val SOLO_CONTROLLER_UNIT_UPDATED = PACKAGE_NAME + ".SOLO_CONTROLLER_UNIT_UPDATED"
}
