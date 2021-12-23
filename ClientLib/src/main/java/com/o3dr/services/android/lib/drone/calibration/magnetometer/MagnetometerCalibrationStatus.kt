package com.o3dr.services.android.lib.drone.calibration.magnetometer

import com.o3dr.services.android.lib.drone.property.DroneAttribute
import android.os.Parcel
import android.os.Parcelable
import java.util.ArrayList
import java.util.HashMap

/** Created by Fredia Huya-Kouadio on 5/4/15. */
class MagnetometerCalibrationStatus : DroneAttribute {
    private val calibrationProgressTracker: MutableMap<Int, MagnetometerCalibrationProgress> = HashMap()
    private val calibrationResultTracker: MutableMap<Int, MagnetometerCalibrationResult> = HashMap()
    private val compassList: MutableList<Int> = ArrayList()
    var isCalibrationCancelled = false

    constructor() {}

    fun addCalibrationProgress(progress: MagnetometerCalibrationProgress?) {
        if (progress != null) {
            val compassId = progress.compassId
            calibrationProgressTracker[compassId] = progress
            compassList.add(compassId)
        }
    }

    fun addCalibrationResult(result: MagnetometerCalibrationResult?) {
        if (result != null) {
            val compassId = result.compassId
            calibrationResultTracker[result.compassId] = result
            compassList.add(compassId)
        }
    }

    val compassIds: List<Int>
        get() = compassList

    fun getCalibrationProgress(compassId: Int): MagnetometerCalibrationProgress? {
        return calibrationProgressTracker[compassId]
    }

    fun getCalibrationResult(compassId: Int): MagnetometerCalibrationResult? {
        return calibrationResultTracker[compassId]
    }

    val isCalibrationComplete: Boolean
        get() {
            for (compassId in compassList) {
                if (!calibrationResultTracker.containsKey(compassId)) return false
            }
            return true
        }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        val progressList: List<MagnetometerCalibrationProgress> = ArrayList(calibrationProgressTracker
                .values)
        dest.writeTypedList(progressList)
        val resultList: List<MagnetometerCalibrationResult> = ArrayList(calibrationResultTracker.values)
        dest.writeTypedList(resultList)
        dest.writeByte(if (isCalibrationCancelled) 1.toByte() else 0.toByte())
    }

    private constructor(input: Parcel) {
        val progressList: List<MagnetometerCalibrationProgress> = ArrayList()
        input.readTypedList(progressList, MagnetometerCalibrationProgress.CREATOR)
        for (progress in progressList) {
            addCalibrationProgress(progress)
        }
        val resultList: List<MagnetometerCalibrationResult> = ArrayList()
        input.readTypedList(resultList, MagnetometerCalibrationResult.CREATOR)
        for (result in resultList) addCalibrationResult(result)
        isCalibrationCancelled = input.readByte().toInt() != 0
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<MagnetometerCalibrationStatus> = object : Parcelable.Creator<MagnetometerCalibrationStatus> {
            override fun createFromParcel(source: Parcel): MagnetometerCalibrationStatus? {
                return MagnetometerCalibrationStatus(source)
            }

            override fun newArray(size: Int): Array<MagnetometerCalibrationStatus?> {
                return arrayOfNulls(size)
            }
        }
    }
}
