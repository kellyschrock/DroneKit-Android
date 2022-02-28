package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.DoJump
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Created by Toby on 7/31/2015.
 */
class DoJump : MissionItem, MissionItem.Command, Parcelable {
    var waypoint = 0
    var repeatCount = 0

    constructor() : super(MissionItemType.DO_JUMP) {}
    constructor(copy: DoJump) : this() {
        waypoint = copy.waypoint
        repeatCount = copy.repeatCount
    }

    protected constructor(`in`: Parcel) : super(`in`) {
        waypoint = `in`.readInt()
        repeatCount = `in`.readInt()
    }

    override fun toString(): String {
        return "DoJump{" +
                "repeatCount=" + repeatCount +
                ", waypoint=" + waypoint +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is DoJump) return false
        if (!super.equals(o)) return false
        val doJump = o
        return if (waypoint != doJump.waypoint) false else repeatCount == doJump.repeatCount
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + waypoint
        result = 31 * result + repeatCount
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeInt(waypoint)
        dest.writeInt(repeatCount)
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun clone(): MissionItem {
        return DoJump(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<DoJump> = object : Parcelable.Creator<DoJump> {
            override fun createFromParcel(`in`: Parcel): DoJump? {
                return DoJump(`in`)
            }

            override fun newArray(size: Int): Array<DoJump?> {
                return arrayOfNulls(size)
            }
        }
    }
}
