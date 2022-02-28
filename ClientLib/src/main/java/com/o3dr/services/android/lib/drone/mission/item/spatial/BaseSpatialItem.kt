package com.o3dr.services.android.lib.drone.mission.item.spatial

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.SpatialItem
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/** Created by fhuya on 11/6/14. */
abstract class BaseSpatialItem : MissionItem, SpatialItem, Parcelable {
    override var coordinate: LatLongAlt? = null

    protected constructor(type: MissionItemType?, coordinate: LatLongAlt? = null) : super(type) {
        this.coordinate = coordinate!!
    }

    protected constructor(type: MissionItemType?): super(type) {
        this.coordinate = null
    }

    protected constructor(copy: BaseSpatialItem) : this(copy.type, if (copy.coordinate == null) null else LatLongAlt(copy.coordinate!!)) {}

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is BaseSpatialItem) return false
        if (!super.equals(o)) return false
        return !if (coordinate != null) coordinate != o.coordinate else o.coordinate != null
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + if (coordinate != null) coordinate.hashCode() else 0
        return result
    }

    override fun toString(): String {
        return "BaseSpatialItem{" +
                "coordinate=" + coordinate +
                '}'
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeParcelable(coordinate, flags)
    }

    protected constructor(input: Parcel) : super(input) {
        coordinate = input.readParcelable(LatLongAlt::class.java.classLoader)
    }
}
