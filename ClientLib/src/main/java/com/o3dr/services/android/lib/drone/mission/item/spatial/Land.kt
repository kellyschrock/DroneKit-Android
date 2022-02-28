package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class Land : BaseSpatialItem, Parcelable {
    constructor() : super(MissionItemType.LAND, LatLongAlt(0.0, 0.0, 0.0)) {}
    constructor(copy: Land?) : super(copy!!) {}
    private constructor(input: Parcel) : super(input) {}

    override fun toString(): String {
        return "Land{ " + super.toString() + " }"
    }

    override fun clone(): MissionItem {
        return Land(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Land> = object : Parcelable.Creator<Land> {
            override fun createFromParcel(source: Parcel): Land? {
                return Land(source)
            }

            override fun newArray(size: Int): Array<Land?> {
                return arrayOfNulls(size)
            }
        }
    }
}
