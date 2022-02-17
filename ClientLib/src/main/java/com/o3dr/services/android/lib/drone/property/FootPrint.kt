package com.o3dr.services.android.lib.drone.property

import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.util.MathUtils
import android.os.Parcel
import android.os.Parcelable
import java.util.ArrayList

/**
 * Created by fhuya on 11/11/14.
 */
class FootPrint : DroneAttribute {
    var meanGSD = 0.0
    var vertexInGlobalFrame: List<LatLong> = ArrayList()
        private set

    constructor() {}
    constructor(meanGSD: Double, vertex: List<LatLong>) {
        this.meanGSD = meanGSD
        vertexInGlobalFrame = vertex
    }

    fun setVertex(vertex: List<LatLong>) {
        vertexInGlobalFrame = vertex
    }

    val lateralSize: Double
        get() = (MathUtils.getDistance2D(vertexInGlobalFrame[0], vertexInGlobalFrame[1])
                + MathUtils.getDistance2D(vertexInGlobalFrame[2], vertexInGlobalFrame[3])) / 2

    val longitudinalSize: Double
        get() = (MathUtils.getDistance2D(vertexInGlobalFrame[0], vertexInGlobalFrame[3])
                + MathUtils.getDistance2D(vertexInGlobalFrame[1], vertexInGlobalFrame[2])) / 2

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(meanGSD)
        dest.writeTypedList(vertexInGlobalFrame)
    }

    private constructor(input: Parcel) {
        meanGSD = input.readDouble()
        input.readTypedList(vertexInGlobalFrame, LatLong.CREATOR)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<FootPrint> = object : Parcelable.Creator<FootPrint> {
            override fun createFromParcel(source: Parcel): FootPrint? {
                return FootPrint(source)
            }

            override fun newArray(size: Int): Array<FootPrint?> {
                return arrayOfNulls(size)
            }
        }
    }
}
