package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.EpmGripper
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Created by fhuya on 11/6/14.
 */
class EpmGripper : MissionItem, MissionItem.Command, Parcelable {
    var isRelease = false

    constructor() : super(MissionItemType.EPM_GRIPPER) {}
    constructor(copy: EpmGripper) : this() {
        isRelease = copy.isRelease
    }

    override fun toString(): String {
        return "EpmGripper{" +
                "release=" + isRelease +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is EpmGripper) return false
        if (!super.equals(o)) return false
        return isRelease == o.isRelease
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + if (isRelease) 1 else 0
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeByte(if (isRelease) 1.toByte() else 0.toByte())
    }

    private constructor(`in`: Parcel) : super(`in`) {
        isRelease = `in`.readByte().toInt() != 0
    }

    override fun clone(): MissionItem {
        return EpmGripper(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<EpmGripper> = object : Parcelable.Creator<EpmGripper> {
            override fun createFromParcel(source: Parcel): EpmGripper? {
                return EpmGripper(source)
            }

            override fun newArray(size: Int): Array<EpmGripper?> {
                return arrayOfNulls(size)
            }
        }
    }
}
