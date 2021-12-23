package com.o3dr.android.client.utils.video

import java.nio.ByteBuffer

/**
 * Created by fhuya on 12/4/14.
 */
class NaluChunk(payloadCount: Int, payloadSize: Int, payloadInitData: ByteArray?) {
    @JvmField
    val payloads: Array<ByteBuffer?> = arrayOfNulls(payloadCount)
    @JvmField
    var type = 0
    @JvmField
    var sequenceNumber = 0
    @JvmField
    var flags = 0
    @JvmField
    var presentationTime: Long = 0

    companion object {
        @JvmField
        val START_CODE = byteArrayOf(0, 0, 0, 1)
        const val SPS_NAL_TYPE = 7
        const val PPS_NAL_TYPE = 8
    }

    init {
        for (i in 0 until payloadCount) {
            payloads[i] = ByteBuffer.allocate(payloadSize)
            if (payloadInitData != null) {
                payloads[i]?.apply {
                    put(payloadInitData)
                    mark()
                }
            }
        }
    }
}
