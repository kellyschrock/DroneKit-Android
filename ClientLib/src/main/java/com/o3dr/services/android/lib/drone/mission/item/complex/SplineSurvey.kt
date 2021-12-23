package com.o3dr.services.android.lib.drone.mission.item.complex

import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 */
class SplineSurvey : Survey {
    constructor() : super(MissionItemType.SPLINE_SURVEY) {}
    constructor(copy: Survey?) : this() {
        copy(copy)
    }

    constructor(copy: SplineSurvey?) : this(copy as Survey?) {}
    private constructor(input: Parcel) : super(input) {}

    override fun toString(): String {
        return "SplineSurvey{" + super.toString() + "}"
    }

    override fun clone(): MissionItem {
        return SplineSurvey(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SplineSurvey> = object : Parcelable.Creator<SplineSurvey> {
            override fun createFromParcel(source: Parcel): SplineSurvey? {
                return SplineSurvey(source)
            }

            override fun newArray(size: Int): Array<SplineSurvey?> {
                return arrayOfNulls(size)
            }
        }
    }
}
