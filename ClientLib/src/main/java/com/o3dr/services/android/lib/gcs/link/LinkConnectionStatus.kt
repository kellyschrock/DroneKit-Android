package com.o3dr.services.android.lib.gcs.link

import android.os.Bundle
import android.os.Parcel
import android.os.Parcelable
import android.support.annotation.IntDef
import android.support.annotation.StringDef

/**
 * Conveys information about the link connection state.
 *
 *
 * This value is returned in the [com.o3dr.android.client.Drone.notifyAttributeUpdated] as the
 * extra value [com.o3dr.services.android.lib.gcs.link.LinkEventExtra.EXTRA_CONNECTION_STATUS]
 * when the attribute event is [com.o3dr.services.android.lib.gcs.link.LinkEvent.LINK_STATE_UPDATED]
 */
class LinkConnectionStatus : Parcelable {
    /**
     * The possible status codes that notifies what state the link connection is in.
     */
    @StringDef(CONNECTED, CONNECTING, DISCONNECTED, FAILED)
    @kotlin.annotation.Retention(AnnotationRetention.SOURCE)
    annotation class StatusCode

    /**
     * The possible failure codes that can be retrieved from the [.getExtras] using key
     * [.EXTRA_ERROR_CODE]. A [LinkConnectionStatus.FailureCode]
     * is guaranteed when [.FAILED] occurs.
     *
     */
    @IntDef(SYSTEM_UNAVAILABLE.toLong(), LINK_UNAVAILABLE.toLong(), PERMISSION_DENIED.toLong(), INVALID_CREDENTIALS.toLong(), TIMEOUT.toLong(), ADDRESS_IN_USE.toLong(), UNKNOWN.toLong())
    @kotlin.annotation.Retention(AnnotationRetention.SOURCE)
    annotation class FailureCode

    /**
     * @return Returns the status of the link connection. This value is one of [LinkConnectionStatus.StatusCode]
     */
    @get:StatusCode
    @StatusCode
    val statusCode: String?

    /**
     * @return Returns a [Bundle] with additional information about the link connection.
     */
    val extras: Bundle?

    constructor(@StatusCode statusCode: String?, extras: Bundle?) {
        this.statusCode = statusCode
        this.extras = extras
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(statusCode)
        dest.writeBundle(extras)
    }

    private constructor(input: Parcel) {
        @StatusCode val statusCode = input.readString()
        this.statusCode = statusCode
        extras = input.readBundle()
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other == null || javaClass != other.javaClass) {
            return false
        }
        val that = other as LinkConnectionStatus
        return if (if (statusCode != null) statusCode != that.statusCode else that.statusCode != null) {
            false
        } else !if (extras != null) extras != that.extras else that.extras != null
    }

    override fun hashCode(): Int {
        var result = if (statusCode != null) statusCode.hashCode() else 0
        result = 31 * result + if (extras != null) extras.hashCode() else 0
        return result
    }

    override fun toString(): String {
        return "ConnectionResult{" +
                "mStatusCode='" + statusCode + '\'' +
                ", mExtras=" + extras +
                '}'
    }

    companion object {
        /**
         * Key that is used to get the [FailureCode] from [.getExtras]
         * to determine what link connection error occurred. This will always be populated when [.FAILED] occurs.
         */
        const val EXTRA_ERROR_CODE = "extra_error_code"

        /**
         * Key that is used to retrieve information from [.getExtras] about why the link connection
         * failure occurred. This value may be populated when [.FAILED] occurs, or can be null.
         */
        const val EXTRA_ERROR_MSG = "extra_error_message"

        /**
         * Key that is used to retrieve the time a link connection occurred from [.getExtras].
         * This is guaranteed when [.CONNECTED] occurs.
         */
        const val EXTRA_CONNECTION_TIME = "extra_connection_time"
        const val CONNECTED = "CONNECTED"
        const val CONNECTING = "CONNECTING"
        const val DISCONNECTED = "DISCONNECTED"
        const val FAILED = "FAILED"

        /**
         * The system does not allow the requested connection type.
         */
        const val SYSTEM_UNAVAILABLE = -1

        /**
         * Requested device to connect to is not available. See [.EXTRA_ERROR_MSG] for more information.
         */
        const val LINK_UNAVAILABLE = -2

        /**
         * Unable to access the requested connection type.
         */
        const val PERMISSION_DENIED = -3

        /**
         * The provided credentials could not be authorized.
         */
        const val INVALID_CREDENTIALS = -4

        /**
         * A timeout attempting to connect to device has occurred.
         */
        const val TIMEOUT = -5

        /**
         * A [java.net.BindException] occurred, determining that the requested address is in use.
         */
        const val ADDRESS_IN_USE = -6

        /**
         * All errors that are not one of the listed [LinkConnectionStatus.FailureCode]s.
         * This is usually due to a device system failure.
         */
        const val UNKNOWN = -7

        @JvmField
        val CREATOR: Parcelable.Creator<LinkConnectionStatus> = object : Parcelable.Creator<LinkConnectionStatus> {
            override fun createFromParcel(source: Parcel): LinkConnectionStatus? {
                return LinkConnectionStatus(source)
            }

            override fun newArray(size: Int): Array<LinkConnectionStatus?> {
                return arrayOfNulls(size)
            }
        }

        /**
         * Helper method to generate the generic [.FAILED] [LinkConnectionStatus]
         * @param failureCode Of type [LinkConnectionStatus.FailureCode]
         * @param errMsg A message that gives more information to the client about the error. This can be null.
         *
         * @return Returns a [LinkConnectionStatus] with statusCode [.FAILED]
         */
        @JvmStatic
        fun newFailedConnectionStatus(@FailureCode failureCode: Int, errMsg: String?): LinkConnectionStatus {
            val extras = Bundle().apply {
                putInt(EXTRA_ERROR_CODE, failureCode)
                putString(EXTRA_ERROR_MSG, errMsg)
            }

            return LinkConnectionStatus(FAILED, extras)
        }
    }
}
