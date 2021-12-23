package com.o3dr.services.android.lib.gcs.returnToMe

import android.os.Parcel
import android.os.Parcelable
import android.support.annotation.IntDef
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import java.lang.annotation.Retention
import java.lang.annotation.RetentionPolicy

/**
 * Created by Fredia Huya-Kouadio on 9/22/15.
 */
class ReturnToMeState : DroneAttribute {
    @IntDef(STATE_IDLE.toLong(),
            STATE_USER_LOCATION_UNAVAILABLE.toLong(),
            STATE_USER_LOCATION_INACCURATE.toLong(),
            STATE_WAITING_FOR_VEHICLE_GPS.toLong(),
            STATE_UPDATING_HOME.toLong(),
            STATE_ERROR_UPDATING_HOME.toLong())

    @kotlin.annotation.Retention(AnnotationRetention.SOURCE)
    annotation class ReturnToMeStates

    private var originalHomeLocation: LatLongAlt? = null
    private var currentHomeLocation: LatLongAlt? = null

    @get:ReturnToMeStates
    @ReturnToMeStates
    var state = STATE_IDLE.toLong()

    constructor() {}
    constructor(@ReturnToMeStates state: Int) {
        this.state = state.toLong()
    }

    fun getCurrentHomeLocation(): LatLongAlt? {
        return currentHomeLocation
    }

    fun setCurrentHomeLocation(currentHomeLocation: LatLongAlt?) {
        this.currentHomeLocation = currentHomeLocation?.let { LatLongAlt(it) }
    }

    fun getOriginalHomeLocation(): LatLongAlt? {
        return originalHomeLocation
    }

    fun setOriginalHomeLocation(originalHomeLocation: LatLongAlt?) {
        this.originalHomeLocation = originalHomeLocation?.let { LatLongAlt(it) }
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeLong(state)
        dest.writeParcelable(originalHomeLocation, 0)
        dest.writeParcelable(currentHomeLocation, 0)
    }

    protected constructor(input: Parcel) {
        @ReturnToMeStates val temp = input.readLong()
        state = temp
        originalHomeLocation = input.readParcelable(LatLongAlt::class.java.classLoader)
        currentHomeLocation = input.readParcelable(LatLongAlt::class.java.classLoader)
    }

    companion object {
        const val STATE_IDLE = 0
        const val STATE_USER_LOCATION_UNAVAILABLE = 1
        const val STATE_USER_LOCATION_INACCURATE = 2
        const val STATE_WAITING_FOR_VEHICLE_GPS = 3
        const val STATE_UPDATING_HOME = 4
        const val STATE_ERROR_UPDATING_HOME = 5

        @JvmField
        val CREATOR: Parcelable.Creator<ReturnToMeState> = object : Parcelable.Creator<ReturnToMeState> {
            override fun createFromParcel(source: Parcel): ReturnToMeState? {
                return ReturnToMeState(source)
            }

            override fun newArray(size: Int): Array<ReturnToMeState?> {
                return arrayOfNulls(size)
            }
        }
    }
}
