package com.o3dr.android.client.apis.solo

import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import com.o3dr.services.android.lib.model.AbstractCommandListener
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.android.client.apis.Api
import com.o3dr.services.android.lib.drone.companion.solo.action.SoloActions
import com.o3dr.services.android.lib.model.action.Action

/**
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
abstract class SoloApi protected constructor(protected val drone: Drone) : Api() {
    /**
     * Sends a message to the solo vehicle.
     * @param messagePacket TLV message data.
     * @param listener Register a callback to receive update of the command execution status.
     */
    protected open fun sendMessage(messagePacket: TLVPacket?, listener: AbstractCommandListener?) {
        val params = Bundle().apply {
            putParcelable(SoloActions.EXTRA_MESSAGE_DATA, messagePacket)
        }

        drone.performAsyncActionOnDroneThread(Action(SoloActions.ACTION_SEND_MESSAGE, params), listener)
    }
}
