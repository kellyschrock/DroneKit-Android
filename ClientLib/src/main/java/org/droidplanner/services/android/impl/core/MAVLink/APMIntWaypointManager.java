package org.droidplanner.services.android.impl.core.MAVLink;

import android.os.Handler;
import android.util.Log;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.common.msg_mission_ack;
import com.MAVLink.common.msg_mission_count;
import com.MAVLink.common.msg_mission_current;
import com.MAVLink.common.msg_mission_item;
import com.MAVLink.common.msg_mission_item_int;
import com.MAVLink.common.msg_mission_item_reached;
import com.MAVLink.common.msg_mission_request;
import com.MAVLink.common.msg_mission_request_int;
import com.MAVLink.enums.MAV_MISSION_TYPE;
import java.util.ArrayList;
import java.util.List;
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnWaypointManagerListener;
import org.droidplanner.services.android.impl.core.drone.DroneVariable;
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone;
import org.droidplanner.services.android.impl.core.mission.MissionItemConvert;
import timber.log.Timber;

/**
 * Class to manage the communication of waypoints to the MAV.
 * <p/>
 * Should be initialized with a MAVLink Object, so the manager can send messages
 * via the MAV link. The function processMessage must be called with every new
 * MAV Message.
 */
public class APMIntWaypointManager extends DroneVariable implements IWaypointManager<msg_mission_item_int> {
    static final String TAG = APMIntWaypointManager.class.getSimpleName();

    private static final long TIMEOUT = 15000; //ms
    private static final int RETRY_LIMIT = 3;

    private int retryTracker = 0;

    private int readIndex;
    private int writeIndex;
    private int retryIndex;
    private OnWaypointManagerListener wpEventListener;

    WaypointStates state = WaypointStates.IDLE;

    /**
     * waypoint witch is currently being written
     */

    private final Handler watchdog;

    private final Runnable watchdogCallback = new Runnable() {
        @Override
        public void run() {
            if (processTimeOut(++retryTracker))
                watchdog.postDelayed(this, TIMEOUT);
        }
    };

    public APMIntWaypointManager(MavLinkDrone drone, Handler handler) {
        super(drone);
        this.watchdog = handler;
    }

    @Override
    public void setWaypointManagerListener(OnWaypointManagerListener wpEventListener) {
        this.wpEventListener = wpEventListener;
    }

    private void startWatchdog() {
        stopWatchdog();

        retryTracker = 0;
        this.watchdog.postDelayed(watchdogCallback, TIMEOUT);
    }

    private void stopWatchdog() {
        this.watchdog.removeCallbacks(watchdogCallback);
    }

    /**
     * Try to receive all waypoints from the MAV.
     * <p/>
     * If all runs well the callback will return the list of waypoints.
     */
    @Override
    public void getWaypoints() {
        // ensure that WPManager is not doing anything else
        if (state != WaypointStates.IDLE)
            return;

        doBeginWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD);
        readIndex = -1;
        state = WaypointStates.READ_REQUEST;
        MavLinkWaypoint.requestWaypointsList(myDrone);

        startWatchdog();
    }

    /**
     * Write a list of waypoints to the MAV.
     * <p/>
     * The callback will return the status of this operation
     *
     * @param data waypoints to be written
     */

    @Override
    public void writeWaypoints(List<msg_mission_item_int> data) {
        Log.v(TAG, "writeWaypoints(): state=" + state);

        // ensure that WPManager is not doing anything else
        if (state != WaypointStates.IDLE)
            return;

        if ((mission != null)) {
            doBeginWaypointEvent(WaypointEvent_Type.WP_UPLOAD);
            mission.clear();
            mission.addAll(data);
            writeIndex = 0;
            state = WaypointStates.WRITING_WP_COUNT;
            Log.v(TAG, "sendWaypointCount(): mission.size=" + mission.size());

            MavLinkWaypoint.sendWaypointCount(myDrone, mission.size(), MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);

            startWatchdog();
        }
    }

    /**
     * Sets the current waypoint in the MAV
     * <p/>
     * The callback will return the status of this operation
     */
    @Override
    public void setCurrentWaypoint(int i) {
        if ((mission != null)) {
            MavLinkWaypoint.sendSetCurrentWaypoint(myDrone, (short) i);
        }
    }

    /**
     * Callback for when a waypoint has been reached
     *
     * @param wpNumber number of the completed waypoint
     */
    @Override
    public void onWaypointReached(int wpNumber) {
    }

    /**
     * Callback for a change in the current waypoint the MAV is heading for
     *
     * @param seq number of the updated waypoint
     */
    private void onCurrentWaypointUpdate(int seq) {
    }

    /**
     * number of waypoints to be received, used when reading waypoints
     */
    private int waypointCount;
    /**
     * list of waypoints used when writing or receiving
     */
    private List<msg_mission_item_int> mission = new ArrayList<>();

    /**
     * Try to process a Mavlink message if it is a mission related message
     *
     * @param msg Mavlink message to process
     * @return Returns true if the message has been processed
     */
    @Override
    public boolean processMessage(MAVLinkMessage msg) {
//        Log.v(TAG, String.format("processMessage(): %s", msg.name()));

        switch (state) {
            default:
            case IDLE:
                break;

            case READ_REQUEST:
//                Log.v(TAG, "READ_REQUEST");
                if (msg.msgid == msg_mission_count.MAVLINK_MSG_ID_MISSION_COUNT) {
                    waypointCount = ((msg_mission_count) msg).count;
                    Log.v(TAG, "Got a mission count at least");
                    mission.clear();
                    startWatchdog();
                    MavLinkWaypoint.requestWayPointInt(myDrone, mission.size());
                    state = WaypointStates.READING_WP;
                    return true;
                }
                break;

            case READING_WP:
                switch(msg.msgid) {
                    case msg_mission_item_int.MAVLINK_MSG_ID_MISSION_ITEM_INT: {
                        startWatchdog();
                        processReceivedWaypoint((msg_mission_item_int) msg);
                        doWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD, readIndex + 1, waypointCount);
                        if (mission.size() < waypointCount) {
                            MavLinkWaypoint.requestWayPointInt(myDrone, mission.size());
                        } else {
                            stopWatchdog();
                            state = WaypointStates.IDLE;
                            MavLinkWaypoint.sendAck(myDrone);
                            myDrone.getMission().onMissionReceived(MissionItemConvert.toMissionItems(mission));
                            doEndWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD);
                        }
                        return true;
                    }
                }
                break;

            case WRITING_WP_COUNT:
                state = WaypointStates.WRITING_WP;
            case WRITING_WP:
                switch(msg.msgid) {
                    case msg_mission_request.MAVLINK_MSG_ID_MISSION_REQUEST: {
                        Log.v(TAG, "got MISSION_REQUEST");

                        startWatchdog();
                        processWaypointToSend((msg_mission_request) msg);
                        doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission.size());
                        return true;
                    }

                    case msg_mission_request_int.MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                        Log.v(TAG, "got MISSION_REQUEST");

                        startWatchdog();

                        final msg_mission_request ask = new msg_mission_request();
                        ask.mission_type = ((msg_mission_request)msg).mission_type;
                        ask.seq = ((msg_mission_request)msg).seq;
                        ask.target_component = ((msg_mission_request)msg).target_component;
                        ask.target_system = ((msg_mission_request)msg).target_system;

                        processWaypointToSend(ask);
                        doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission.size());
                        return true;
                    }
                }

                break;

            case WAITING_WRITE_ACK:
                if (msg.msgid == msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK) {
                    msg_mission_ack ack = (msg_mission_ack)msg;
                    Timber.d("Got MISSION_ACK: type=%d", ack.type);

                    stopWatchdog();
                    myDrone.getMission().onWriteWaypoints((msg_mission_ack) msg);
                    state = WaypointStates.IDLE;
                    doEndWaypointEvent(WaypointEvent_Type.WP_UPLOAD);
                    return true;
                }
                break;
        }

        if (msg.msgid == msg_mission_item_reached.MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
            onWaypointReached(((msg_mission_item_reached) msg).seq);
            return true;
        }

        if (msg.msgid == msg_mission_current.MAVLINK_MSG_ID_MISSION_CURRENT) {
            onCurrentWaypointUpdate(((msg_mission_current) msg).seq);
            return true;
        }
        return false;
    }

    @Override
    public boolean processTimeOut(int mTimeOutCount) {

        // If max retry is reached, set state to IDLE. No more retry.
        if (mTimeOutCount >= RETRY_LIMIT) {
            state = WaypointStates.IDLE;
            doWaypointEvent(WaypointEvent_Type.WP_TIMED_OUT, retryIndex, RETRY_LIMIT);
            return false;
        }

        retryIndex++;
        doWaypointEvent(WaypointEvent_Type.WP_RETRY, retryIndex, RETRY_LIMIT);

        switch (state) {
            default:
            case IDLE:
                break;

            case READ_REQUEST:
                MavLinkWaypoint.requestWaypointsList(myDrone);
                break;

            case READING_WP:
                if (mission.size() < waypointCount) { // request last lost WP
                    MavLinkWaypoint.requestWayPoint(myDrone, mission.size());
                }
                break;

            case WRITING_WP_COUNT:
                MavLinkWaypoint.sendWaypointCount(myDrone, mission.size(), MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                break;

            case WRITING_WP:
                // Log.d("TIMEOUT", "re Write Msg: " + String.valueOf(writeIndex));
                if (writeIndex < mission.size()) {
                    myDrone.getMavClient().sendMessage(mission.get(writeIndex), null);
                }
                break;

            case WAITING_WRITE_ACK:
                myDrone.getMavClient().sendMessage(mission.get(mission.size() - 1), null);
                break;
        }

        return true;
    }

    private void processWaypointToSend(msg_mission_request msg) {
        Log.v(TAG, "processWaypointToSend(): msg=" + msg);

        /*
         * Log.d("TIMEOUT", "Write Msg: " + String.valueOf(msg.seq));
		 */
        writeIndex = msg.seq;
        msg_mission_item_int item = mission.get(writeIndex);
        Log.v(TAG, String.format("processWaypointToSend(): item=%s", item));

        item.isMavlink2 = false;
        item.mission_type = 0;
        item.target_system = myDrone.getSysid();
        item.target_component = myDrone.getCompid();

        // This is how it's supposed to work.
        if(msg.sysid != 0 && msg.compid != 0) {
            item.target_system = (short)msg.sysid;
            item.target_component = (short)msg.compid;
        }

        Timber.d("send item %s", item);

        myDrone.getMavClient().sendMessage(item, null);

        if (writeIndex + 1 >= mission.size()) {
            state = WaypointStates.WAITING_WRITE_ACK;
        }
    }

    private void processReceivedWaypoint(msg_mission_item_int msg) {
//        Log.v(TAG, String.format("processReceivedWaypoint(): %s", msg));
		/*
		 * Log.d("TIMEOUT", "Read Last/Curr: " + String.valueOf(readIndex) + "/"
		 * + String.valueOf(msg.seq));
		 */
        // in case of we receive the same WP again after retry
        if (msg.seq <= readIndex)
            return;

        readIndex = msg.seq;

        mission.add(msg);
    }

    private void doBeginWaypointEvent(WaypointEvent_Type wpEvent) {
        retryIndex = 0;

        if (wpEventListener == null)
            return;

        wpEventListener.onBeginWaypointEvent(wpEvent);
    }

    private void doEndWaypointEvent(WaypointEvent_Type wpEvent) {
        if (retryIndex > 0)// if retry successful, notify that we now continue
            doWaypointEvent(WaypointEvent_Type.WP_CONTINUE, retryIndex, RETRY_LIMIT);

        retryIndex = 0;

        if (wpEventListener == null)
            return;

        wpEventListener.onEndWaypointEvent(wpEvent);
    }

    private void doWaypointEvent(WaypointEvent_Type wpEvent, int index, int count) {
        retryIndex = 0;

        if (wpEventListener == null)
            return;

        wpEventListener.onWaypointEvent(wpEvent, index, count);
    }

}
