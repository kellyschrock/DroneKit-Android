package com.o3dr.services.android.lib.drone.mission.item.complex

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.ComplexItem
import com.o3dr.services.android.lib.util.MathUtils
import java.util.*

/**
 */
open class Survey : MissionItem, ComplexItem<Survey?>, Parcelable {
    var surveyDetail: SurveyDetail? = SurveyDetail()
    var polygonArea = 0.0
    var polygonPoints: List<LatLong> = ArrayList()
    var gridPoints: List<LatLong>? = ArrayList()
    var cameraLocations: List<LatLong>? = ArrayList()
    private var cameraElevations: MutableList<LatLongAlt>? = ArrayList()
    var centerElevation = 0.0
    var isValid = false
    /**
     * @since 2.8.1
     * @return true if the camera trigger should be started before reaching the first survey waypoint.
     */
    /**
     * Enable to start the camera trigger before reaching the first survey waypoint.
     * @since 2.8.1
     * @param startCameraBeforeFirstWaypoint
     */
    var isStartCameraBeforeFirstWaypoint = false

    constructor() : this(MissionItemType.SURVEY) {}
    protected constructor(type: MissionItemType?) : super(type) {}
    constructor(copy: Survey) : this() {
        copy(copy)
    }

    override fun copy(input: Survey?) {
        input?.let { source ->
            surveyDetail = SurveyDetail(source.surveyDetail)
            polygonArea = source.polygonArea
            polygonPoints = copyPointsList(source.polygonPoints)
            gridPoints = copyPointsList(source.gridPoints)
            cameraLocations = copyPointsList(source.cameraLocations)
            cameraElevations!!.addAll(source.cameraElevations!!)
            isValid = source.isValid
            isStartCameraBeforeFirstWaypoint = source.isStartCameraBeforeFirstWaypoint
        }
    }

    private fun copyPointsList(copy: List<LatLong>?): List<LatLong> {
        val dest: MutableList<LatLong> = ArrayList()

        copy?.let {
            for (itemCopy in it) {
                dest.add(LatLong(itemCopy))
            }
        }

        return dest
    }

    val gridLength: Double
        get() = MathUtils.getPolylineLength(gridPoints!!)

    val numberOfLines: Int
        get() = gridPoints!!.size / 2

    fun getCameraElevations(): List<LatLongAlt> {
        if (cameraElevations == null) cameraElevations = ArrayList()
        return cameraElevations!!
    }

    fun setCameraElevations(elevations: List<LatLongAlt>?) {
        if (cameraElevations == null) cameraElevations = ArrayList()
        cameraElevations!!.clear()
        cameraElevations!!.addAll(elevations!!)
    }

    fun getCameraCount(): Int {
        return cameraLocations!!.size
    }

    override fun toString(): String {
        return "Survey{" +
                "cameraLocations=" + cameraLocations +
                ", surveyDetail=" + surveyDetail +
                ", polygonArea=" + polygonArea +
                ", polygonPoints=" + polygonPoints +
                ", gridPoints=" + gridPoints +
                ", cameraElevations=" + cameraElevations +
                ", isValid=" + isValid +
                ", startCameraBeforeFirstWaypoint=" + isStartCameraBeforeFirstWaypoint +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Survey) return false
        if (!super.equals(o)) return false
        if (java.lang.Double.compare(o.polygonArea, polygonArea) != 0) return false
        if (isValid != o.isValid) return false
        if (isStartCameraBeforeFirstWaypoint != o.isStartCameraBeforeFirstWaypoint) return false
        if (if (surveyDetail != null) surveyDetail != o.surveyDetail else o.surveyDetail != null) return false
        if (if (polygonPoints != null) polygonPoints != o.polygonPoints else o.polygonPoints != null) return false
        if (if (gridPoints != null) gridPoints != o.gridPoints else o.gridPoints != null) return false
        return if (if (cameraElevations != null) cameraElevations != o.cameraElevations else o.cameraElevations != null) false else !if (cameraLocations != null) cameraLocations != o.cameraLocations else o.cameraLocations != null
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + if (surveyDetail != null) surveyDetail.hashCode() else 0
        val temp: Long = java.lang.Double.doubleToLongBits(polygonArea)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (polygonPoints != null) polygonPoints.hashCode() else 0
        result = 31 * result + if (gridPoints != null) gridPoints.hashCode() else 0
        result = 31 * result + if (cameraLocations != null) cameraLocations.hashCode() else 0
        result = 31 * result + if (cameraElevations != null) cameraElevations.hashCode() else 0
        result = 31 * result + if (isValid) 1 else 0
        result = 31 * result + if (isStartCameraBeforeFirstWaypoint) 1 else 0
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeParcelable(surveyDetail, 0)
        dest.writeDouble(polygonArea)
        dest.writeTypedList(polygonPoints)
        dest.writeTypedList(gridPoints)
        dest.writeTypedList(cameraLocations)
        dest.writeTypedList(cameraElevations)
        dest.writeByte(if (isValid) 1.toByte() else 0.toByte())
        dest.writeByte(if (isStartCameraBeforeFirstWaypoint) 1.toByte() else 0.toByte())
    }

    protected constructor(input: Parcel) : super(input) {
        surveyDetail = input.readParcelable(SurveyDetail::class.java.classLoader)
        polygonArea = input.readDouble()
        input.readTypedList(polygonPoints, LatLong.CREATOR)
        input.readTypedList(gridPoints, LatLong.CREATOR)
        input.readTypedList(cameraLocations, LatLong.CREATOR)
        input.readTypedList(cameraElevations, LatLongAlt.CREATOR)
        isValid = input.readByte().toInt() != 0
        isStartCameraBeforeFirstWaypoint = input.readByte().toInt() != 0
    }

    public override fun clone(): MissionItem {
        return Survey(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Survey> = object : Parcelable.Creator<Survey> {
            override fun createFromParcel(source: Parcel): Survey {
                return Survey(source)
            }

            override fun newArray(size: Int): Array<Survey?> {
                return arrayOfNulls(size)
            }
        }
    }

    init {
        surveyDetail?.apply {
            altitude = 50.0
            angle = 0.0
            overlap = 50.0
            sidelap = 60.0
            lockOrientation = false
        }
    }
}
