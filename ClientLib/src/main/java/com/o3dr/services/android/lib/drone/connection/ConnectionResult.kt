package com.o3dr.services.android.lib.drone.connection

import android.os.Parcelable
import android.os.Parcel

@Deprecated("""Use {@link LinkConnectionStatus} instead.
 
  Conveys information if the connection attempt fails.""")
class ConnectionResult : Parcelable {
    val errorCode: Int
    val errorMessage: String?

    constructor(errorCode: Int, errorMessage: String?) {
        this.errorCode = errorCode
        this.errorMessage = errorMessage
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(errorCode)
        dest.writeString(errorMessage)
    }

    private constructor(input: Parcel) {
        errorCode = input.readInt()
        errorMessage = input.readString()
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ConnectionResult) return false
        if (errorCode != other.errorCode) return false
        return !if (errorMessage != null) errorMessage != other.errorMessage else other.errorMessage != null
    }

    override fun hashCode(): Int {
        var result = errorCode
        result = 31 * result + if (errorMessage != null) errorMessage.hashCode() else 0
        return result
    }

    override fun toString(): String {
        return "ConnectionResult{" +
                "mErrorCode=" + errorCode +
                ", mErrorMessage='" + errorMessage + '\'' +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<ConnectionResult> = object : Parcelable.Creator<ConnectionResult> {
            override fun createFromParcel(source: Parcel): ConnectionResult? {
                return ConnectionResult(source)
            }

            override fun newArray(size: Int): Array<ConnectionResult?> {
                return arrayOfNulls(size)
            }
        }
    }
}
