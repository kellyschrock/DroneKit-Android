package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class CameraTrigger : MissionItem, MissionItem.Command, Parcelable {
    var triggerDistance = 0.0

    constructor() : super(MissionItemType.CAMERA_TRIGGER) {}
    constructor(copy: CameraTrigger) : super(MissionItemType.CAMERA_TRIGGER) {
        triggerDistance = copy.triggerDistance
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is CameraTrigger) return false
        if (!super.equals(o)) return false
        return java.lang.Double.compare(o.triggerDistance, triggerDistance) == 0
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long
        temp = java.lang.Double.doubleToLongBits(triggerDistance)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "CameraTrigger{" +
                "triggerDistance=" + triggerDistance +
                '}'
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(triggerDistance)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        triggerDistance = `in`.readDouble()
    }

    override fun clone(): MissionItem {
        return CameraTrigger(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<CameraTrigger> = object : Parcelable.Creator<CameraTrigger> {
            override fun createFromParcel(source: Parcel): CameraTrigger? {
                return CameraTrigger(source)
            }

            override fun newArray(size: Int): Array<CameraTrigger?> {
                return arrayOfNulls(size)
            }
        }
    }
}
