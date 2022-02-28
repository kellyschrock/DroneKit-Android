package com.o3dr.services.android.lib.drone.mission.item.complex

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.ComplexItem
import com.o3dr.services.android.lib.drone.mission.item.spatial.BaseSpatialItem
import java.util.*

/**
 *
 */
class StructureScanner : BaseSpatialItem, ComplexItem<StructureScanner?>, Parcelable {
    var radius = 10.0
    var heightStep = 5.0
    var stepsCount = 2
    var isCrossHatch = false
    var surveyDetail: SurveyDetail? = SurveyDetail()
    var path: List<LatLong>? = ArrayList()

    constructor() : super(MissionItemType.STRUCTURE_SCANNER) {}
    constructor(copy: StructureScanner) : super(copy) {
        copy(copy)
    }

    override fun copy(input: StructureScanner?) {
        input?.let { source ->
            radius = source.radius
            heightStep = source.heightStep
            stepsCount = source.stepsCount
            isCrossHatch = source.isCrossHatch
            surveyDetail = SurveyDetail(source.surveyDetail)
            path = copyPointsList(source.path)
        }
    }

    private fun copyPointsList(copy: List<LatLong>?): List<LatLong> {
        val dest: MutableList<LatLong> = ArrayList()
        for (itemCopy in copy!!) {
            dest.add(LatLong(itemCopy))
        }
        return dest
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(radius)
        dest.writeDouble(heightStep)
        dest.writeInt(stepsCount)
        dest.writeByte(if (isCrossHatch) 1.toByte() else 0.toByte())
        dest.writeParcelable(surveyDetail, 0)
        dest.writeTypedList(path)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        radius = `in`.readDouble()
        heightStep = `in`.readDouble()
        stepsCount = `in`.readInt()
        isCrossHatch = `in`.readByte().toInt() != 0
        surveyDetail = `in`.readParcelable(SurveyDetail::class.java.classLoader)
        `in`.readTypedList(path, LatLong.CREATOR)
    }

    override fun toString(): String {
        return "StructureScanner{" +
                "crossHatch=" + isCrossHatch +
                ", radius=" + radius +
                ", heightStep=" + heightStep +
                ", stepsCount=" + stepsCount +
                ", surveyDetail=" + surveyDetail +
                ", path=" + path +
                ", " + super.toString() +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is StructureScanner) return false
        if (!super.equals(o)) return false
        if (java.lang.Double.compare(o.radius, radius) != 0) return false
        if (java.lang.Double.compare(o.heightStep, heightStep) != 0) return false
        if (stepsCount != o.stepsCount) return false
        if (isCrossHatch != o.isCrossHatch) return false
        return if (if (surveyDetail != null) surveyDetail != o.surveyDetail else o.surveyDetail != null) false else !if (path != null) path != o.path else o.path != null
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        var temp: Long = java.lang.Double.doubleToLongBits(radius)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(heightStep)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + stepsCount
        result = 31 * result + if (isCrossHatch) 1 else 0
        result = 31 * result + if (surveyDetail != null) surveyDetail.hashCode() else 0
        result = 31 * result + if (path != null) path.hashCode() else 0
        return result
    }

    public override fun clone(): MissionItem {
        return StructureScanner(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<StructureScanner> = object : Parcelable.Creator<StructureScanner> {
            override fun createFromParcel(source: Parcel): StructureScanner? {
                return StructureScanner(source)
            }

            override fun newArray(size: Int): Array<StructureScanner?> {
                return arrayOfNulls(size)
            }
        }
    }
}
