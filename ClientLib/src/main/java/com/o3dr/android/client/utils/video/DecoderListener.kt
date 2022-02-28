package com.o3dr.android.client.utils.video

/** Created by fhuya on 12/4/14. */
interface DecoderListener {
    fun onDecodingStarted()
    fun onDecodingError()
    fun onDecodingEnded()
}
