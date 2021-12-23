package com.o3dr.services.android.lib.drone.companion.solo.button

import android.os.Parcelable
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonTypes
import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonPacket
import android.os.Parcel
import android.util.Log
import java.nio.BufferUnderflowException
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Created by djmedina on 4/15/15.
 */
class ButtonPacket : Parcelable {
    var timestamp = -1.0
        private set
    var buttonId: Byte = -1
        private set
    var pressedMask: Short = -1
        private set

    private val byteBuffer: ByteBuffer
    private var eventType: Byte = -1

    constructor(pressedMask: Short, buttonId: Byte, eventType: Byte, timestamp: Double) {
        this.pressedMask = pressedMask
        this.buttonId = buttonId
        this.eventType = eventType
        this.timestamp = timestamp
        byteBuffer = ByteBuffer.allocate(ButtonTypes.MESSAGE_LENGTH)
        byteBuffer.order(BYTE_ORDER)
    }

    /**
     * Construc a ButtonPacket from a Parcel
     * @param in a Parcel of a button
     */
    private constructor(`in`: Parcel) {
        timestamp = `in`.readDouble()
        eventType = `in`.readByte()
        buttonId = `in`.readByte()
        pressedMask = `in`.readValue(Short::class.javaPrimitiveType!!.classLoader) as Short
        byteBuffer = ByteBuffer.allocate(ButtonTypes.MESSAGE_LENGTH)
        byteBuffer.order(BYTE_ORDER)
    }

    fun getEventType(): Int {
        return eventType.toInt()
    }

    fun toBytes(): ByteArray {
        byteBuffer.clear()
        /**
         * Message format
         * Byte    Size    Description
         * 0       8       Timestamp, since epoch
         * 8       1       Button ID
         * 9       1       Button event
         * 10      2       Buttons-pressed mask
         * 12 (packet length)
         */
        byteBuffer.putDouble(timestamp)
        byteBuffer.put(buttonId)
        byteBuffer.put(eventType)
        byteBuffer.putShort(pressedMask)
        val bytes = ByteArray(byteBuffer.position())
        byteBuffer.rewind()
        byteBuffer[bytes]
        return bytes
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(timestamp)
        dest.writeByte(eventType)
        dest.writeByte(buttonId)
        dest.writeValue(pressedMask)
    }

    companion object {
        val BYTE_ORDER = ByteOrder.LITTLE_ENDIAN
        private val TAG = ButtonPacket::class.java.simpleName
        @JvmStatic
        fun parseButtonPacket(packetBuffer: ByteBuffer?): ButtonPacket? {
            if (packetBuffer == null || packetBuffer.limit() <= 0) return null
            val originalOrder = packetBuffer.order()
            /**
             * Message format
             * Byte    Size    Description
             * 0       8       Timestamp, usec since epoch
             * 8       1       Button ID
             * 9       1       Button event
             * 10      2       Buttons-pressed mask
             * 12 (packet length)
             */
            return try {
                packetBuffer.order(BYTE_ORDER)
                val timestamp = packetBuffer.double
                val buttonId = packetBuffer.get()
                val eventType = packetBuffer.get()
                val pressedMask = packetBuffer.short
                ButtonPacket(pressedMask, buttonId, eventType, timestamp)
            } catch (e: BufferUnderflowException) {
                Log.e(TAG, "Invalid data for button packet", e)
                null
            } finally {
                packetBuffer.order(originalOrder)
            }
        }

        @JvmField
        val CREATOR: Parcelable.Creator<ButtonPacket?> = object : Parcelable.Creator<ButtonPacket?> {
            override fun createFromParcel(source: Parcel): ButtonPacket? {
                return ButtonPacket(source)
            }

            override fun newArray(size: Int): Array<ButtonPacket?> {
                return arrayOfNulls(size)
            }
        }
    }
}
