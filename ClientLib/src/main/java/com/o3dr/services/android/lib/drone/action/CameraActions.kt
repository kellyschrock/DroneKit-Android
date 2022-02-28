package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.drone.action.CameraActions

/**
 * Created by Fredia Huya-Kouadio on 7/31/15.
 */
object CameraActions {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.companion.solo.action.camera"

    const val ACTION_START_VIDEO_STREAM = PACKAGE_NAME + ".START_VIDEO_STREAM"
    const val EXTRA_VIDEO_DISPLAY = "extra_video_display"
    const val EXTRA_VIDEO_TAG = "extra_video_tag"
    const val EXTRA_VIDEO_PROPERTIES = "extra_video_properties"
    const val EXTRA_VIDEO_PROPS_UDP_PORT = "extra_video_props_udp_port"
    const val EXTRA_VIDEO_PROPS_UDP_IP = "extra_video_props_udp_ip"
    const val EXTRA_VIDEO_ENABLE_LOCAL_RECORDING = "extra_video_enable_local_recording"
    const val EXTRA_VIDEO_LOCAL_RECORDING_FILENAME = "extra_video_local_recording_filename"
    const val ACTION_STOP_VIDEO_STREAM = "$PACKAGE_NAME.STOP_VIDEO_STREAM"
}
