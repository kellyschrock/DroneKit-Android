package com.o3dr.services.android.lib.gcs.follow

import android.os.Parcelable
import android.os.Parcel
import com.o3dr.services.android.lib.gcs.follow.FollowLocationSource

/**
 * Location source for Follow
 */
enum class FollowLocationSource(val label: String) : Parcelable {
    NONE("None"),
    INTERNAL("Device GPS"),
    CLIENT_SPECIFIED("Client Specified")
    ;

    override fun toString(): String {
        return label
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(name)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<FollowLocationSource> = object : Parcelable.Creator<FollowLocationSource> {
            override fun createFromParcel(source: Parcel): FollowLocationSource? {
                return valueOf(source.readString())
            }

            override fun newArray(size: Int): Array<FollowLocationSource?> {
                return arrayOfNulls(size)
            }
        }
    }
}
