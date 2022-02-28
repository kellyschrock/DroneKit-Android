package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class ReturnToLaunch : MissionItem, MissionItem.Command, Parcelable {
    var returnAltitude = 0.0

    constructor() : super(MissionItemType.RETURN_TO_LAUNCH) {}
    constructor(copy: ReturnToLaunch) : this() {
        returnAltitude = copy.returnAltitude
    }

    override fun toString(): String {
        return "ReturnToLaunch{" +
                "returnAltitude=" + returnAltitude +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is ReturnToLaunch) return false
        if (!super.equals(o)) return false
        return java.lang.Double.compare(o.returnAltitude, returnAltitude) == 0
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long
        temp = java.lang.Double.doubleToLongBits(returnAltitude)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(returnAltitude)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        returnAltitude = `in`.readDouble()
    }

    override fun clone(): MissionItem {
        return ReturnToLaunch(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<ReturnToLaunch> = object : Parcelable.Creator<ReturnToLaunch> {
            override fun createFromParcel(source: Parcel): ReturnToLaunch? {
                return ReturnToLaunch(source)
            }

            override fun newArray(size: Int): Array<ReturnToLaunch?> {
                return arrayOfNulls(size)
            }
        }
    }
}
