package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt

/**
 * Created by fhuya on 11/5/14.
 */
class GuidedState : DroneAttribute {
    private var state = 0
    var coordinate: LatLongAlt? = null

    constructor() {}
    constructor(state: Int, coordinate: LatLongAlt?) {
        this.state = state
        this.coordinate = coordinate
    }

    val isActive: Boolean
        get() = state == STATE_ACTIVE
    val isIdle: Boolean
        get() = state == STATE_IDLE
    val isInitialized: Boolean
        get() = state != STATE_UNINITIALIZED

    fun setState(state: Int) {
        this.state = state
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(state)
        dest.writeParcelable(coordinate, flags)
    }

    private constructor(input: Parcel) {
        state = input.readInt()
        coordinate = input.readParcelable(LatLongAlt::class.java.classLoader)
    }

    companion object {
        const val STATE_UNINITIALIZED = 0
        const val STATE_IDLE = 1
        const val STATE_ACTIVE = 2

        @JvmField
        val CREATOR: Parcelable.Creator<GuidedState> = object : Parcelable.Creator<GuidedState> {
            override fun createFromParcel(source: Parcel): GuidedState? {
                return GuidedState(source)
            }

            override fun newArray(size: Int): Array<GuidedState?> {
                return arrayOfNulls(size)
            }
        }
    }
}
