package org.droidplanner.services.android.impl.core.MAVLink;

import android.os.Handler;
import android.util.Log;

import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.common.msg_mission_ack;
import com.MAVLink.common.msg_mission_clear_all;
import com.MAVLink.common.msg_mission_count;
import com.MAVLink.common.msg_mission_current;
import com.MAVLink.common.msg_mission_item;
import com.MAVLink.common.msg_mission_item_int;
import com.MAVLink.common.msg_mission_item_reached;
import com.MAVLink.common.msg_mission_request;
import com.MAVLink.common.msg_mission_request_int;
import com.MAVLink.common.msg_mission_set_current;
import com.MAVLink.enums.MAV_FRAME;
import com.MAVLink.enums.MAV_MISSION_TYPE;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnWaypointManagerListener;
import org.droidplanner.services.android.impl.core.drone.DroneVariable;
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone;

import java.util.ArrayList;
import java.util.List;

import timber.log.Timber;

/**
 * Class to manage the communication of waypoints to the MAV.
 * <p/>
 * Should be initialized with a MAVLink Object, so the manager can send messages
 * via the MAV link. The function processMessage must be called with every new
 * MAV Message.
 */
public class PX4WaypointManager extends DroneVariable implements IWaypointManager {
    static final String TAG = PX4WaypointManager.class.getSimpleName();

    private static final long TIMEOUT = 12000; //ms
    private static final int RETRY_LIMIT = 10;

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

    public PX4WaypointManager(MavLinkDrone drone, Handler handler) {
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
        Log.v(TAG, "getWaypoints(): state=" + state);

        // ensure that WPManager is not doing anything else
        if (state != WaypointStates.IDLE)
            return;

        doBeginWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD);
        readIndex = -1;
        setState(WaypointStates.READ_REQUEST);
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
    private File logFile = null;

    @Override
    public void writeWaypoints(List<msg_mission_item> data) {
        Timber.d("writeWaypoints(): data=%s", data);

        final SimpleDateFormat sdf = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss");
        final File root = myDrone.getContext().getExternalFilesDir(null);
        final File dir = new File(root, "missionupload");

        boolean makeOutputFile = true;
        if(!dir.exists()) {
            if(!dir.mkdirs()) {
                Log.e(TAG, String.format("Unable to create directory %s", dir.getAbsolutePath()));
                makeOutputFile = false;
            }
        }

        if(makeOutputFile) {
            logFile = new File(dir, String.format("%s.log", sdf.format(new java.util.Date())));
        }

        // ensure that WPManager is not doing anything else
        if (state != WaypointStates.IDLE) {
            Log.v(TAG, String.format("Abort: State in %s", state));
            return;
        }

        if ((mission != null)) {
            doBeginWaypointEvent(WaypointEvent_Type.WP_UPLOAD);
            mission.clear();
            mission.addAll(data);

            writeIndex = 0;
            setState(WaypointStates.WRITING_WP_COUNT);
            Log.v(TAG, "sendWaypointCount()");

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
    private List<msg_mission_item> mission = new ArrayList<msg_mission_item>();

    /**
     * Try to process a Mavlink message if it is a mission related message
     *
     * @param msg Mavlink message to process
     * @return Returns true if the message has been processed
     */
    @Override
    public boolean processMessage(MAVLinkMessage msg) {
        switch (state) {
            default:
            case IDLE:
                break;

            case READ_REQUEST:
                Timber.d("READ_REQUEST");
                if (msg.msgid == msg_mission_count.MAVLINK_MSG_ID_MISSION_COUNT) {
                    waypointCount = ((msg_mission_count) msg).count;
                    mission.clear();
                    startWatchdog();
                    MavLinkWaypoint.requestWayPoint(myDrone, mission.size());
                    setState(WaypointStates.READING_WP);
                    return true;
                }
                break;

            case READING_WP:
                if (msg.msgid == msg_mission_item.MAVLINK_MSG_ID_MISSION_ITEM) {
                    Timber.d("READING_WP getting mission item: %s", msg);

                    startWatchdog();
                    processReceivedWaypoint((msg_mission_item) msg);
                    doWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD, readIndex + 1, waypointCount);
                    if (mission.size() < waypointCount) {
                        MavLinkWaypoint.requestWayPoint(myDrone, mission.size());
                    } else {
                        stopWatchdog();
                        setState(WaypointStates.IDLE);
                        MavLinkWaypoint.sendAck(myDrone);
                        myDrone.getMission().onPX4MissionReceived(mission);
                        doEndWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD);
                    }
                    return true;
                }
                break;

            case WRITING_WP_COUNT:
                Timber.d("WRITING_WP_COUNT");
                setState(WaypointStates.WRITING_WP);
                // FALL THROUGH
            case WRITING_WP:
                switch(msg.msgid) {
                    case msg_mission_request.MAVLINK_MSG_ID_MISSION_REQUEST: {
                        Log.v(TAG, "got " + msg);
                        logToFile(String.format("Vehicle <- %s", msg));

                        startWatchdog();
                        processWaypointToSend((msg_mission_request) msg);
                        doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission.size());
                        return true;
                    }

                    case msg_mission_request_int.MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                        Log.v(TAG, "got " + msg);
                        logToFile(String.format("Vehicle <- %s", msg));

                        startWatchdog();
                        processWaypointToSend((msg_mission_request_int) msg);
                        doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission.size());
                        return true;
                    }

                    default: break;
                }

                break;

            case WAITING_WRITE_ACK:
                Timber.d("WAITING_WRITE_ACK");
                if (msg.msgid == msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK) {
                    logToFile(String.format("Vehicle <- %s", msg));

                    msg_mission_ack ack = (msg_mission_ack)msg;
                    Timber.d("Got MISSION_ACK: type=%d", ack.type);

                    stopWatchdog();
                    myDrone.getMission().onWriteWaypoints((msg_mission_ack) msg);
                    setState(WaypointStates.IDLE);
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
    public boolean processTimeOut(int retryCount) {
        Log.v(TAG, String.format("processTimeout(%d)", retryCount));

        // If max retry is reached, set state to IDLE. No more retry.
        if (retryCount >= RETRY_LIMIT) {
            setState(WaypointStates.IDLE);
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
        /*
         * Log.d("TIMEOUT", "Write Msg: " + String.valueOf(msg.seq));
		 */
        writeIndex = msg.seq;
        msg_mission_item item = mission.get(writeIndex);

        item.isMavlink2 = false;
        item.seq = msg.seq;
        item.mission_type = 0;
        item.target_system = myDrone.getSysid();
        item.target_component = myDrone.getCompid();

        // This is what it's supposed to do, not the above.
        item.target_system = (short)msg.sysid;
        item.target_component = (short)msg.compid;

//        Log.v(TAG, String.format("Send MISSION_ITEM seq=%d", item.seq));
        Log.v(TAG, String.format("Send MISSION_ITEM %s", item));
        logToFile(String.format("Client -> %s", item.toString()));

        myDrone.getMavClient().sendMessage(item, null);

        if (writeIndex + 1 >= mission.size()) {
            setState(WaypointStates.WAITING_WRITE_ACK);
        }
    }

    private void sendFirstWaypoint() {
        Log.v(TAG, "sendFirstWaypoint()");

        msg_mission_item msg = mission.get(writeIndex);

        writeIndex = msg.seq;
        msg_mission_item_int item = new msg_mission_item_int();
        msg_mission_item src = mission.get(writeIndex);

        item.isMavlink2 = false;
        item.mission_type = 0;
        item.seq = msg.seq;
        item.target_system = myDrone.getSysid();
        item.target_component = myDrone.getCompid();
        item.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        item.command = src.command;
        item.current = src.current;
        item.autocontinue = src.autocontinue;
        item.param1 = src.param1;
        item.param2 = src.param2;
        item.param3 = src.param3;
        item.param4 = src.param4;
        item.x = (int)(src.x  * 1E7);
        item.y = (int)(src.y  * 1E7);
        item.z = src.z;

        Timber.d("send item %s", item);
        logToFile(String.format("Client -> %s", item));

        myDrone.getMavClient().sendMessage(item, null);

        if (writeIndex + 1 >= mission.size()) {
            setState(WaypointStates.WAITING_WRITE_ACK);
        }
    }

    private void processWaypointToSend(msg_mission_request_int msg) {
        /*
         * Log.d("TIMEOUT", "Write Msg: " + String.valueOf(msg.seq));
		 */
        writeIndex = msg.seq;
        msg_mission_item_int item = new msg_mission_item_int();
        msg_mission_item src = mission.get(writeIndex);

        item.isMavlink2 = false;
        item.mission_type = 0;
        item.seq = msg.seq;
        item.target_system = myDrone.getSysid();
        item.target_component = myDrone.getCompid();
        item.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        item.command = src.command;
        item.current = src.current;
        item.autocontinue = src.autocontinue;
        item.param1 = src.param1;
        item.param2 = src.param2;
        item.param3 = src.param3;
        item.param4 = src.param4;
        item.x = (int)(src.x  * 1E7);
        item.y = (int)(src.y  * 1E7);
        item.z = src.z;

        Timber.d("send item %s", item);

        myDrone.getMavClient().sendMessage(item, null);

        if (writeIndex + 1 >= mission.size()) {
            setState(WaypointStates.WAITING_WRITE_ACK);
        }
    }

    private void sendClearAll() {
        final msg_mission_clear_all clear = new msg_mission_clear_all();
        clear.target_system = myDrone.getSysid();
        clear.target_component = myDrone.getCompid();
        clear.mission_type = 0;
        myDrone.getMavClient().sendMessage(clear, null);
    }

    private void sendMissionSetCurrent() {
        Log.v(TAG, "Send MISSION_SET_CURRENT");
        final msg_mission_set_current msg = new msg_mission_set_current();
        msg.seq = 0;
        msg.target_system = myDrone.getSysid();
        msg.target_component = myDrone.getCompid();
        myDrone.getMavClient().sendMessage(msg, null);
    }

    private void processReceivedWaypoint(msg_mission_item msg) {
		/*
		 * Log.d("TIMEOUT", "Read Last/Curr: " + String.valueOf(readIndex) + "/"
		 * + String.valueOf(msg.seq));
		 */
        // in case of we receive the same WP again after retry
        if (msg.seq <= readIndex)
            return;

        readIndex = msg.seq;

        mission.add(msg);
        Log.v(TAG, String.format("processReceivedWaypoint(): %d items", mission.size()));
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

    private void setState(WaypointStates state) {
        Timber.d("setState(%s)", state);
        this.state = state;
    }

    private void logToFile(String line) {
        if(logFile != null) {
            try {
                final String filename = logFile.getAbsolutePath();
                Log.v("Waypoint", "filename=" + filename);

                final FileWriter writer = new FileWriter(filename, true);
                final BufferedWriter bw = new BufferedWriter(writer);
                try {
                    bw.write(line);
                    bw.newLine();
                } finally {
                    bw.flush();
                    bw.close();
                }
            } catch(IOException ex) {
                Log.e("Waypoint", ex.getMessage(), ex);
            }
        }
    }
}
