package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.ChangeSpeed
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Created by fhuya on 11/6/14.
 */
class ChangeSpeed : MissionItem, MissionItem.Command, Parcelable {
    var speed = 0.0

    constructor() : super(MissionItemType.CHANGE_SPEED) {}
    constructor(copy: ChangeSpeed) : super(MissionItemType.CHANGE_SPEED) {
        speed = copy.speed
    }

    override fun toString(): String {
        return "ChangeSpeed{" +
                "speed=" + speed +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is ChangeSpeed) return false
        if (!super.equals(o)) return false
        return java.lang.Double.compare(o.speed, speed) == 0
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long
        temp = java.lang.Double.doubleToLongBits(speed)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(speed)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        speed = `in`.readDouble()
    }

    override fun clone(): MissionItem {
        return ChangeSpeed(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<ChangeSpeed> = object : Parcelable.Creator<ChangeSpeed> {
            override fun createFromParcel(source: Parcel): ChangeSpeed? {
                return ChangeSpeed(source)
            }

            override fun newArray(size: Int): Array<ChangeSpeed?> {
                return arrayOfNulls(size)
            }
        }
    }
}
