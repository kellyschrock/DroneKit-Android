package com.o3dr.services.android.lib.drone.companion.solo.button

/**
 * Created by djmedina on 4/15/15.
 */
object ButtonTypes {
    const val MESSAGE_LENGTH = 12

    /**
     * Button IDs
     */
    const val BUTTON_POWER = 0
    const val BUTTON_FLY = 1
    const val BUTTON_RTL = 2
    const val BUTTON_LOITER = 3
    const val BUTTON_A = 4
    const val BUTTON_B = 5
    const val BUTTON_PRESET_1 = 6
    const val BUTTON_PRESET_2 = 7
    const val BUTTON_CAMERA_CLICK = 8

    /**
     * Event types
     */
    const val BUTTON_EVENT_PRESS = 0
    const val BUTTON_EVENT_RELEASE = 1
    const val BUTTON_EVENT_CLICK_RELEASE = 2
    const val BUTTON_EVENT_HOLD = 3
    const val BUTTON_EVENT_LONG_HOLD = 4
    const val BUTTON_EVENT_DOUBLE_CLICK = 5
}
