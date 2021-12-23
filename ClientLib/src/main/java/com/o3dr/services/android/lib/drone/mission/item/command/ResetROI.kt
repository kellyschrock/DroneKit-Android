package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Reset the current region of interest lock.
 * Created by Fredia Huya-Kouadio on 10/20/15.
 * @since 2.6.8
 */
class ResetROI : MissionItem, MissionItem.Command {
    constructor() : super(MissionItemType.RESET_ROI) {}
    constructor(copy: ResetROI?) : this() {}

    override fun clone(): MissionItem {
        return ResetROI(this)
    }

    override fun toString(): String {
        return "ResetROI{}"
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
    }

    protected constructor(input: Parcel) : super(input) {}

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<ResetROI> = object : Parcelable.Creator<ResetROI> {
            override fun createFromParcel(source: Parcel): ResetROI? {
                return ResetROI(source)
            }

            override fun newArray(size: Int): Array<ResetROI?> {
                return arrayOfNulls(size)
            }
        }
    }
}
