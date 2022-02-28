package com.o3dr.services.android.lib.gcs.follow

import com.o3dr.services.android.lib.drone.property.DroneAttribute
import android.os.Bundle
import com.o3dr.services.android.lib.gcs.follow.FollowType
import com.o3dr.services.android.lib.gcs.follow.FollowState
import android.os.Parcel
import android.os.Parcelable

/** Created by fhuya on 11/5/14.  */
class FollowState : DroneAttribute {
    var state = 0
    var params: Bundle? = null
        private set
    var mode: FollowType? = null

    constructor() {}

    constructor(state: Int, mode: FollowType?, modeParams: Bundle?) {
        this.state = state
        params = modeParams
        this.mode = mode
    }

    val isEnabled: Boolean
        get() = state == STATE_RUNNING || state == STATE_START

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(state)
        dest.writeBundle(params)
        dest.writeInt(if (mode == null) -1 else mode!!.ordinal)
    }

    private constructor(`in`: Parcel) {
        state = `in`.readInt()
        params = `in`.readBundle()
        val tmpMode = `in`.readInt()
        mode = if (tmpMode == -1) null else FollowType.values()[tmpMode]
    }

    companion object {
        const val STATE_INVALID = 0
        const val STATE_DRONE_NOT_ARMED = 1
        const val STATE_DRONE_DISCONNECTED = 2
        const val STATE_START = 3
        const val STATE_RUNNING = 4
        const val STATE_END = 5

        @JvmField
        val CREATOR: Parcelable.Creator<FollowState> = object : Parcelable.Creator<FollowState> {
            override fun createFromParcel(source: Parcel): FollowState? {
                return FollowState(source)
            }

            override fun newArray(size: Int): Array<FollowState?> {
                return arrayOfNulls(size)
            }
        }
    }
}
