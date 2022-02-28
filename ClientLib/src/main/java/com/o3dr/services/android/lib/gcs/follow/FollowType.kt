package com.o3dr.services.android.lib.gcs.follow

import android.os.Parcelable
import com.o3dr.services.android.lib.gcs.follow.FollowType
import android.os.Parcel
import java.util.ArrayList

/**
 * Created by fhuya on 11/5/14.
 */
enum class FollowType(private val typeLabel: String) : Parcelable {
    LEASH("Leash"),
    LEAD("Lead"),
    RIGHT("Right"),
    LEFT("Left"),
    CIRCLE("Circle"),
    ABOVE("Above") {
        override fun hasParam(paramKey: String?): Boolean {
            return false
        }
    },
    GUIDED_SCAN("Guided Scan") {
        override fun hasParam(paramKey: String?): Boolean {
            return when (paramKey) {
                EXTRA_FOLLOW_ROI_TARGET -> true
                else -> false
            }
        }
    },
    LOOK_AT_ME("Look At Me") {
        override fun hasParam(paramKey: String?): Boolean {
            return false
        }
    },
    SOLO_SHOT("Solo Follow Shot") {
        override fun hasParam(paramKey: String?): Boolean {
            return false
        }
    };

    open fun hasParam(paramKey: String?): Boolean {
        return when (paramKey) {
            EXTRA_FOLLOW_RADIUS -> true
            EXTRA_FOLLOW_ROI_TARGET -> false
            else -> false
        }
    }

    override fun toString(): String {
        return typeLabel
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(name)
    }

    companion object {
        const val EXTRA_FOLLOW_RADIUS = "extra_follow_radius"
        const val EXTRA_FOLLOW_ROI_TARGET = "extra_follow_roi_target"

        fun getFollowTypes(includeAdvanced: Boolean): List<FollowType> {
            val followTypes: MutableList<FollowType> = ArrayList()
            followTypes.add(LEASH)
            followTypes.add(LEAD)
            followTypes.add(RIGHT)
            followTypes.add(LEFT)
            followTypes.add(CIRCLE)
            followTypes.add(ABOVE)
            followTypes.add(GUIDED_SCAN)
            followTypes.add(LOOK_AT_ME)
            return followTypes
        }

        @JvmField
        val CREATOR: Parcelable.Creator<FollowType> = object : Parcelable.Creator<FollowType> {
            override fun createFromParcel(source: Parcel): FollowType? {
                return valueOf(source.readString())
            }

            override fun newArray(size: Int): Array<FollowType?> {
                return arrayOfNulls(size)
            }
        }
    }
}
