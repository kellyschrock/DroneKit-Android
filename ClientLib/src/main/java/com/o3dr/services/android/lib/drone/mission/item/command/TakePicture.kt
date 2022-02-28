package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class TakePicture : MissionItem, MissionItem.Command, Parcelable {
    constructor() : super(MissionItemType.TAKE_PICTURE) {}
    constructor(copy: TakePicture?) : super(MissionItemType.TAKE_PICTURE) {}

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is TakePicture) return false
        if (!super.equals(o)) return false
        val that = o
        return true
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long
        temp = java.lang.Double.doubleToLongBits(0.0)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "TakePicture"
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
    }

    private constructor(`in`: Parcel) : super(`in`) {}

    override fun clone(): MissionItem {
        return TakePicture(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<TakePicture> = object : Parcelable.Creator<TakePicture> {
            override fun createFromParcel(source: Parcel): TakePicture? {
                return TakePicture(source)
            }

            override fun newArray(size: Int): Array<TakePicture?> {
                return arrayOfNulls(size)
            }
        }
    }
}
