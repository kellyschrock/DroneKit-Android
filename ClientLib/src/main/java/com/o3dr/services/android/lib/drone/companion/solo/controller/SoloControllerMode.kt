package com.o3dr.services.android.lib.drone.companion.solo.controller

import android.support.annotation.IntDef
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode
import java.lang.annotation.Retention
import java.lang.annotation.RetentionPolicy

/**
 * Defines the available controller modes.
 * Created by Fredia Huya-Kouadio on 7/13/15.
 */
object SoloControllerMode {
    /**
     * Unknown controller mode.
     */
    const val UNKNOWN_MODE = 0

    /**
     * Controller mode 1:
     * - Left stick: pitch/yaw
     * - Right stick: throttle/roll
     */
    const val MODE_1 = 1

    /**
     * Controller mode 2:
     * - Left stick: throttle/yaw
     * - right stick: pitch/roll
     */
    const val MODE_2 = 2

    @IntDef(UNKNOWN_MODE.toLong(), MODE_1.toLong(), MODE_2.toLong())
    @Retention(RetentionPolicy.SOURCE)
    annotation class ControllerMode
}
