package com.o3dr.services.android.lib.util

import android.os.Parcelable
import android.os.Parcel
import com.o3dr.services.android.lib.util.ParcelableUtils

/**
 * Utilities functions for parcelable objects.
 */
object ParcelableUtils {
    /**
     * Marshall a parcelable object to a byte array
     * @param parcelable
     * @return
     */
    @JvmStatic
    fun marshall(parcelable: Parcelable): ByteArray {
        val parcel = Parcel.obtain()
        parcelable.writeToParcel(parcel, 0)
        val bytes = parcel.marshall()
        parcel.recycle() // not sure if needed or a good idea
        return bytes
    }

    /**
     * Unmarshall a parcel object from a byte array.
     * @param bytes
     * @return
     */
    @JvmStatic
    private fun unmarshall(bytes: ByteArray): Parcel {
        val parcel = Parcel.obtain()
        parcel.unmarshall(bytes, 0, bytes.size)
        parcel.setDataPosition(0) // this is extremely important!
        return parcel
    }

    /**
     * Unmarshall a parcelable instance from a byte array.
     * @param bytes
     * @param creator
     * @param <T>
     * @return
    </T> */
    @JvmStatic
    fun <T> unmarshall(bytes: ByteArray, creator: Parcelable.Creator<T>): T {
        val parcel = unmarshall(bytes)
        val result = creator.createFromParcel(parcel)
        parcel.recycle()
        return result
    }

    /**
     * Unmarshall a parcel object from a byte array.
     * @param bytes
     * @return
     */
    @JvmStatic
    private fun unmarshall(bytes: ByteArray, offset: Int, length: Int): Parcel {
        val parcel = Parcel.obtain()
        parcel.unmarshall(bytes, offset, length)
        parcel.setDataPosition(0) // this is extremely important!
        return parcel
    }

    /**
     * Unmarshall a parcelable instance from a byte array.
     * @param bytes
     * @param creator
     * @param <T>
     * @return
    </T> */
    @JvmStatic
    fun <T> unmarshall(bytes: ByteArray, offset: Int, length: Int, creator: Parcelable.Creator<T>): T {
        val parcel = unmarshall(bytes, offset, length)
        val result = creator.createFromParcel(parcel)
        parcel.recycle()
        return result
    }
}
