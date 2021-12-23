package com.o3dr.services.android.lib.drone.mission.item.complex

import android.os.Parcel
import android.os.Parcelable

/**
 * Created by fhuya on 11/7/14.
 */
class SurveyDetail : Parcelable {
    var altitude = 0.0
    var angle = 0.0
    var overlap = 0.0
    var sidelap = 0.0

    /**
     * Lock aircraft's yaw to the angle of the survey
     * @param lockOrientation
     * @since 3.0.0
     */
    var lockOrientation = false

    var cameraDetail: CameraDetail? = null
    var lockYaw = false
    var lockYawAngle = 0.0

    val lateralFootPrint: Double
        get() = (altitude * cameraDetail!!.sensorLateralSize
                / cameraDetail!!.focalLength)

    val longitudinalFootPrint: Double
        get() = (altitude * cameraDetail!!.sensorLongitudinalSize
                / cameraDetail!!.focalLength)

    val groundResolution: Double
        get() = ((altitude * cameraDetail!!.sensorLateralSize / cameraDetail!!.focalLength
                * (altitude * cameraDetail!!.sensorLongitudinalSize
                / cameraDetail!!.focalLength) / (cameraDetail!!.sensorResolution * 1000))
                / 10000)

    val longitudinalPictureDistance: Double
        get() = longitudinalFootPrint * (1 - overlap * .01)

    val lateralPictureDistance: Double
        get() = lateralFootPrint * (1 - sidelap * .01)

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(altitude)
        dest.writeDouble(angle)
        dest.writeDouble(overlap)
        dest.writeDouble(sidelap)
        dest.writeParcelable(cameraDetail, 0)
        dest.writeByte((if (lockOrientation) 1 else 0).toByte())
        dest.writeByte((if (lockYaw) 1 else 0).toByte())
        dest.writeDouble(lockYawAngle)
    }

    constructor() {}
    constructor(other: SurveyDetail?) {
        other?.let { copy ->
            altitude = copy.altitude
            angle = copy.angle
            overlap = copy.overlap
            sidelap = copy.sidelap
            lockOrientation = copy.lockOrientation
            lockYaw = copy.lockYaw
            lockYawAngle = copy.lockYawAngle
            cameraDetail = if (copy.cameraDetail == null) null else CameraDetail(copy.cameraDetail!!)
        }
    }

    private constructor(`in`: Parcel) {
        altitude = `in`.readDouble()
        angle = `in`.readDouble()
        overlap = `in`.readDouble()
        sidelap = `in`.readDouble()
        cameraDetail = `in`.readParcelable(CameraDetail::class.java.classLoader)
        lockOrientation = `in`.readByte().toInt() != 0
        lockYaw = `in`.readByte().toInt() != 0
        lockYawAngle = `in`.readDouble()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SurveyDetail> = object : Parcelable.Creator<SurveyDetail> {
            override fun createFromParcel(source: Parcel): SurveyDetail {
                return SurveyDetail(source)
            }

            override fun newArray(size: Int): Array<SurveyDetail?> {
                return arrayOfNulls(size)
            }
        }
    }
}
