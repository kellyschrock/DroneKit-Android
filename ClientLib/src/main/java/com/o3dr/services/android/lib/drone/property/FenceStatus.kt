package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/**
 * FenceStatus atttribute
 */
class FenceStatus : DroneAttribute {
    var breachTime: Long = 0
    var breachCount = 0
    var breachStatus: Short = 0
    var breachType: Short = 0

    constructor() : super() {}
    constructor(time: Long, count: Int, status: Short, type: Short) : this() {
        breachTime = time
        breachCount = count
        breachStatus = status
        breachType = type
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other == null || javaClass != other.javaClass) return false
        val that = other as FenceStatus
        if (breachTime != that.breachTime) return false
        if (breachCount != that.breachCount) return false
        return if (breachStatus != that.breachStatus) false else breachType == that.breachType
    }

    override fun hashCode(): Int {
        var result = (breachTime xor (breachTime ushr 32)).toInt()
        result = 31 * result + breachCount
        result = 31 * result + breachStatus.toInt()
        result = 31 * result + breachType.toInt()
        return result
    }

    override fun toString(): String {
        return "FenceStatus{" +
                "breachTime=" + breachTime +
                ", breachCount=" + breachCount +
                ", breachStatus=" + breachStatus +
                ", breachType=" + breachType +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeLong(breachTime)
        dest.writeInt(breachCount)
        dest.writeInt(breachStatus.toInt())
        dest.writeInt(breachType.toInt())
    }

    private constructor(input: Parcel) {
        breachTime = input.readLong()
        breachCount = input.readInt()
        breachStatus = input.readInt().toShort()
        breachType = input.readInt().toShort()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<FenceStatus> = object : Parcelable.Creator<FenceStatus> {
            override fun createFromParcel(source: Parcel): FenceStatus? {
                return FenceStatus(source)
            }

            override fun newArray(size: Int): Array<FenceStatus?> {
                return arrayOfNulls(size)
            }
        }
    }
}
