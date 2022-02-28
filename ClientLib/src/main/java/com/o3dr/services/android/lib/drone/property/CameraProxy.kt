package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.complex.CameraDetail
import java.util.*

/**
 * Created by fhuya on 11/30/14.
 */
class CameraProxy : DroneAttribute {
    var cameraDetail: CameraDetail
        private set
    var footPrints: List<FootPrint> = ArrayList()
        private set
    var currentFieldOfView: FootPrint
        private set
    var availableCameraInfos: List<CameraDetail> = ArrayList()
        private set

    constructor(cameraDetail: CameraDetail, currentFieldOfView: FootPrint,
                footPrints: List<FootPrint>, availableCameraInfos: List<CameraDetail>) {
        this.cameraDetail = cameraDetail
        this.currentFieldOfView = currentFieldOfView
        this.footPrints = footPrints
        this.availableCameraInfos = availableCameraInfos
    }

    val lastFootPrint: FootPrint
        get() = footPrints[footPrints.size - 1]

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeParcelable(cameraDetail, 0)
        dest.writeTypedList(footPrints)
        dest.writeParcelable(currentFieldOfView, 0)
        dest.writeTypedList(availableCameraInfos)
    }

    private constructor(input: Parcel) {
        cameraDetail = input.readParcelable(CameraDetail::class.java.classLoader)
        input.readTypedList(footPrints, FootPrint.CREATOR)
        currentFieldOfView = input.readParcelable(FootPrint::class.java.classLoader)
        input.readTypedList(availableCameraInfos, CameraDetail.CREATOR)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<CameraProxy> = object : Parcelable.Creator<CameraProxy> {
            override fun createFromParcel(source: Parcel): CameraProxy? {
                return CameraProxy(source)
            }

            override fun newArray(size: Int): Array<CameraProxy?> {
                return arrayOfNulls(size)
            }
        }
    }
}
