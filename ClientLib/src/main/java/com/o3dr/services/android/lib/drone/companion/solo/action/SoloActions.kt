package com.o3dr.services.android.lib.drone.companion.solo.action

import com.o3dr.services.android.lib.drone.companion.solo.action.SoloActions

/**
 * Created by Fredia Huya-Kouadio on 7/10/15.
 */
object SoloActions {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.companion.solo.action"
    const val ACTION_SEND_MESSAGE = "$PACKAGE_NAME.SEND_MESSAGE"

    /**
     * TLV message object to send to the sololink companion computer.
     *
     * @see {@link com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket}
     */
    const val EXTRA_MESSAGE_DATA = "extra_message_data"
}
