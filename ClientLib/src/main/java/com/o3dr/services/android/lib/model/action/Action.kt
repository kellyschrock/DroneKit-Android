package com.o3dr.services.android.lib.model.action

import android.os.Parcelable
import android.os.Bundle
import android.os.Parcel

/** Wrapper for action exposed by the api. */
class Action : Parcelable {
    var type: String? = null
        private set
    var data: Bundle? = null
        private set

    constructor(actionType: String?) {
        type = actionType
        data = null
    }

    constructor(actionType: String?, actionData: Bundle?) {
        type = actionType
        data = actionData
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(type)
        dest.writeBundle(data)
    }

    fun readFromParcel(source: Parcel) {
        type = source.readString()
        data = source.readBundle()
    }

    private constructor(`in`: Parcel) {
        readFromParcel(`in`)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Action> = object : Parcelable.Creator<Action> {
            override fun createFromParcel(source: Parcel): Action? {
                return Action(source)
            }

            override fun newArray(size: Int): Array<Action?> {
                return arrayOfNulls(size)
            }
        }
    }
}
