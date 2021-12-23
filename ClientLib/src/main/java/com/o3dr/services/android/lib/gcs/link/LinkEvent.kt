package com.o3dr.services.android.lib.gcs.link

import com.o3dr.services.android.lib.gcs.link.LinkEvent

/**
 * Stores all possible link events.
 */
object LinkEvent {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.gcs.link.event"

    /**
     * Notifies what the link connection status currently is.
     *
     * @see {@link LinkEventExtra.EXTRA_CONNECTION_STATUS}
     */
    const val LINK_STATE_UPDATED = "$PACKAGE_NAME.LINK_STATE_UPDATED"
}
