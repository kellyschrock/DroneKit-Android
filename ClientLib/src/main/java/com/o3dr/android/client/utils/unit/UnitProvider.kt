package com.o3dr.android.client.utils.unit

/**
 * Created by fhuya on 1/11/15.
 */
interface UnitProvider {
    fun areaToString(areaInSqMeters: Double): String?
    fun distanceToString(distanceInMeters: Double): String?
    fun speedToString(speedInMetersPerSeconds: Double): String?
    fun electricChargeToString(chargeInmAh: Double): String?

    companion object {
        const val SQUARE_SYMBOL = "\u00B2"
    }
}
