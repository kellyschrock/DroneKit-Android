package org.droidplanner.services.android.impl.core.MAVLink;

import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.common.msg_mission_item;

import org.droidplanner.services.android.impl.core.drone.DroneInterfaces;

import java.util.List;

public interface IWaypointManager<ItemType> {
    enum WaypointStates {
        IDLE, READ_REQUEST, READING_WP, WRITING_WP_COUNT, WRITING_WP, WAITING_WRITE_ACK
    }

    enum WaypointEvent_Type {
        WP_UPLOAD, WP_DOWNLOAD, WP_RETRY, WP_CONTINUE, WP_TIMED_OUT
    }

    void setWaypointManagerListener(DroneInterfaces.OnWaypointManagerListener wpEventListener);

    void getWaypoints();

    void writeWaypoints(List<ItemType> data);

    void setCurrentWaypoint(int i);

    void onWaypointReached(int wpNumber);

    boolean processMessage(MAVLinkMessage msg);

    boolean processTimeOut(int mTimeOutCount);
}
