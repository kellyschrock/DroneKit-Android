package com.o3dr.services.android.lib.gcs.link

import com.o3dr.services.android.lib.gcs.link.LinkEventExtra

/**
 * Holds handles used to retrieve additional information broadcast along a link event.
 *
 * @see {@link LinkEvent}
 */
object LinkEventExtra {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.gcs.link.event.extra"

    /**
     * Used to access the link connection status.
     *
     * @see {@link LinkConnectionStatus}
     *
     * @see {@link com.o3dr.services.android.lib.gcs.link.LinkEvent.LINK_STATE_UPDATED}
     */
    const val EXTRA_CONNECTION_STATUS = "$PACKAGE_NAME.CONNECTION_STATUS"
}
