package com.o3dr.services.android.lib.drone.companion.solo.controller

import android.support.annotation.StringDef
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits
import java.lang.annotation.Retention
import java.lang.annotation.RetentionPolicy

/**
 * Defines the controller's supported unit systems.
 * Created by Fredia Huya-Kouadio on 9/7/15.
 */
object SoloControllerUnits {
    /**
     * Unknown controller unit
     */
    const val UNKNOWN = "unknown"

    /**
     * Metric unit system
     */
    const val METRIC = "metric"

    /**
     * Imperial unit system
     */
    const val IMPERIAL = "imperial"

    @StringDef(UNKNOWN, METRIC, IMPERIAL)
    @Retention(RetentionPolicy.SOURCE)
    annotation class ControllerUnit
}
