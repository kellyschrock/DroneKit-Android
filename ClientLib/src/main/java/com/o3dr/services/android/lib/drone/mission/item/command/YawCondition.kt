package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/10/14.
 */
class YawCondition : MissionItem, MissionItem.Command, Parcelable {
    var angle = 0.0
    var angularSpeed = 0.0
    var isRelative = false

    constructor() : super(MissionItemType.YAW_CONDITION) {}
    constructor(copy: YawCondition) : this() {
        angle = copy.angle
        angularSpeed = copy.angularSpeed
        isRelative = copy.isRelative
    }

    override fun toString(): String {
        return "YawCondition{" +
                "angle=" + angle +
                ", angularSpeed=" + angularSpeed +
                ", isRelative=" + isRelative +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is YawCondition) return false
        if (!super.equals(o)) return false
        val that = o
        if (java.lang.Double.compare(that.angle, angle) != 0) return false
        return if (java.lang.Double.compare(that.angularSpeed, angularSpeed) != 0) false else isRelative == that.isRelative
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        var temp: Long
        temp = java.lang.Double.doubleToLongBits(angle)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(angularSpeed)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (isRelative) 1 else 0
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(angle)
        dest.writeDouble(angularSpeed)
        dest.writeByte(if (isRelative) 1.toByte() else 0.toByte())
    }

    private constructor(`in`: Parcel) : super(`in`) {
        angle = `in`.readDouble()
        angularSpeed = `in`.readDouble()
        isRelative = `in`.readByte().toInt() != 0
    }

    override fun clone(): MissionItem {
        return YawCondition(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<YawCondition> = object : Parcelable.Creator<YawCondition> {
            override fun createFromParcel(source: Parcel): YawCondition? {
                return YawCondition(source)
            }

            override fun newArray(size: Int): Array<YawCondition?> {
                return arrayOfNulls(size)
            }
        }
    }
}
