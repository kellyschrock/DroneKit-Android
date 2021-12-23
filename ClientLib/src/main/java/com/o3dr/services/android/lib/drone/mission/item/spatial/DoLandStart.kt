package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

class DoLandStart : BaseSpatialItem, Parcelable {
    constructor() : super(MissionItemType.DO_LAND_START) {}
    constructor(copy: DoLandStart?) : super(copy!!) {}
    private constructor(input: Parcel) : super(input) {}

    override fun toString(): String {
        return "DoLandStart{ " + super.toString() + " }"
    }

    override fun clone(): MissionItem {
        return DoLandStart(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<DoLandStart> = object : Parcelable.Creator<DoLandStart> {
            override fun createFromParcel(source: Parcel): DoLandStart? {
                return DoLandStart(source)
            }

            override fun newArray(size: Int): Array<DoLandStart?> {
                return arrayOfNulls(size)
            }
        }
    }
}
