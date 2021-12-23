package com.o3dr.services.android.lib.drone.mission.item

import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Created by fhuya on 11/5/14.
 */
abstract class MissionItem : Cloneable, Parcelable {
    interface Command

    interface SpatialItem {
        var coordinate: LatLongAlt?
    }

    interface ComplexItem<T : MissionItem?> {
        fun copy(source: T)
    }

    val type: MissionItemType?

    protected constructor(type: MissionItemType?) {
        this.type = type
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(type!!.ordinal)
    }

    protected constructor(`in`: Parcel) {
        type = MissionItemType.values()[`in`.readInt()]
    }

    abstract override fun clone(): MissionItem
    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is MissionItem) return false
        return type === o.type
    }

    override fun hashCode(): Int {
        return type?.hashCode() ?: 0
    }

    override fun toString(): String {
        return "MissionItem{" +
                "type=" + type +
                '}'
    }
}
