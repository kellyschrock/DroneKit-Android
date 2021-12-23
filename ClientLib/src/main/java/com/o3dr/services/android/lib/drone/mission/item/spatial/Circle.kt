package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class Circle : BaseSpatialItem, Parcelable {
    var radius = 10.0
    var turns = 1

    constructor() : super(MissionItemType.CIRCLE) {}
    constructor(copy: Circle) : super(copy) {
        radius = copy.radius
        turns = copy.turns
    }

    override fun toString(): String {
        return "Circle{" +
                "radius=" + radius +
                ", turns=" + turns +
                ", " + super.toString() +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Circle) return false
        if (!super.equals(o)) return false
        return if (java.lang.Double.compare(o.radius, radius) != 0) false else turns == o.turns
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long = java.lang.Double.doubleToLongBits(radius)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + turns
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(radius)
        dest.writeInt(turns)
    }

    private constructor(input: Parcel) : super(input) {
        radius = input.readDouble()
        turns = input.readInt()
    }

    override fun clone(): MissionItem {
        return Circle(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Circle> = object : Parcelable.Creator<Circle> {
            override fun createFromParcel(source: Parcel): Circle? {
                return Circle(source)
            }

            override fun newArray(size: Int): Array<Circle?> {
                return arrayOfNulls(size)
            }
        }
    }
}
