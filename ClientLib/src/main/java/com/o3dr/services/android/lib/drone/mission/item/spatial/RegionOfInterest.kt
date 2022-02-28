package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Points the nose of the vehicle and camera gimbal at the "region of interest".
 * Created by fhuya on 11/6/14.
 */
class RegionOfInterest : BaseSpatialItem, Parcelable {
    constructor() : super(MissionItemType.REGION_OF_INTEREST) {}
    constructor(copy: RegionOfInterest?) : super(copy!!) {}
    private constructor(input: Parcel) : super(input) {}

    override fun toString(): String {
        return "RegionOfInterest{ " + super.toString() + " }"
    }

    override fun clone(): MissionItem {
        return RegionOfInterest(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<RegionOfInterest> = object : Parcelable.Creator<RegionOfInterest> {
            override fun createFromParcel(source: Parcel): RegionOfInterest? {
                return RegionOfInterest(source)
            }

            override fun newArray(size: Int): Array<RegionOfInterest?> {
                return arrayOfNulls(size)
            }
        }
    }
}
