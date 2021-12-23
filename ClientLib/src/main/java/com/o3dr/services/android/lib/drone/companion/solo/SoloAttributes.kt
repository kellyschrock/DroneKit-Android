package com.o3dr.services.android.lib.drone.companion.solo

import com.o3dr.services.android.lib.drone.companion.solo.SoloAttributes

/**
 * Stores the set of solo attribute types.
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
object SoloAttributes {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.companion.solo.attribute"

    /**
     * Used to access the sololink state.
     */
    const val SOLO_STATE = PACKAGE_NAME + ".SOLO_STATE"

    /**
     * Used to access the sololink gopro state.
     */
    const val SOLO_GOPRO_STATE = PACKAGE_NAME + ".SOLO_GOPRO_STATE"

    /**
     * Used to access the updated sololink gopro state.
     * @since 2.7.0
     */
    const val SOLO_GOPRO_STATE_V2 = PACKAGE_NAME + ".SOLO_GOPRO_STATE_V2"
}
