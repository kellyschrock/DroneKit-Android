package com.o3dr.android.client.apis.solo

import com.o3dr.android.client.Drone
import com.o3dr.android.client.apis.Api.Builder
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import com.o3dr.services.android.lib.model.AbstractCommandListener
import java.util.concurrent.ConcurrentHashMap

/**
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
class SoloMessageApi protected constructor(drone: Drone?) : SoloApi(drone!!) {
    public override fun sendMessage(messagePacket: TLVPacket?, listener: AbstractCommandListener?) {
        super.sendMessage(messagePacket, listener)
    }

    companion object {
        private val apiCache = ConcurrentHashMap<Drone, SoloMessageApi>()
        private val apiBuilder: Builder<SoloMessageApi> = Builder { drone -> SoloMessageApi(drone) }

        fun getApi(drone: Drone?): SoloMessageApi {
            return getApi(drone, apiCache, apiBuilder)
        }
    }
}

