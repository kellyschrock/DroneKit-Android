package org.droidplanner.services.android.impl.core.drone.variables.calibration;

import android.os.Handler;
import android.os.RemoteException;

import android.util.Log;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.common.msg_command_long;
import com.MAVLink.common.msg_statustext;

import com.MAVLink.enums.ACCELCAL_VEHICLE_POS;
import com.MAVLink.enums.MAV_CMD;
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCalibration;
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces;
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType;
import org.droidplanner.services.android.impl.core.drone.DroneVariable;
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone;
import com.o3dr.services.android.lib.model.ICommandListener;
import com.o3dr.services.android.lib.model.SimpleCommandListener;

import java.util.concurrent.atomic.AtomicReference;

import timber.log.Timber;

public class AccelCalibration extends DroneVariable implements DroneInterfaces.OnDroneListener<MavLinkDrone> {
    private static final String TAG = "AccelCalibration";

    private final Runnable onCalibrationStart = new Runnable() {
        @Override
        public void run() {
            final ICommandListener listener = listenerRef.getAndSet(null);
            if (listener != null) {
                try {
                    listener.onSuccess();
                } catch (RemoteException e) {
                    Timber.e(e, e.getMessage());
                }
            }
        }
    };

    private String mavMsg;
    private boolean calibrating;
    private int currVehiclePos = 0;
    private boolean usingVehiclePos = false;

    private final Handler handler;
    private final AtomicReference<ICommandListener> listenerRef = new AtomicReference<>(null);

    public AccelCalibration(MavLinkDrone drone, Handler handler) {
        super(drone);
        this.handler = handler;
        drone.addDroneListener(this);
    }

    public void startCalibration(ICommandListener listener) {
        if (calibrating) {
            if (listener != null) {
                try {
                    listener.onSuccess();
                } catch (RemoteException e) {
                    Timber.e(e, e.getMessage());
                }
            }
            return;
        }

        if (myDrone.getState().isFlying()) {
            calibrating = false;
        } else {
            calibrating = true;
            mavMsg = "";

            listenerRef.set(listener);
            MavLinkCalibration.startAccelerometerCalibration(myDrone, new SimpleCommandListener() {
                @Override
                public void onSuccess() {
                    final ICommandListener listener = listenerRef.getAndSet(null);
                    if (listener != null) {
                        try {
                            listener.onSuccess();
                        } catch (RemoteException e) {
                            Timber.e(e, e.getMessage());
                        }
                    }
                }

                @Override
                public void onError(int executionError) {
                    final ICommandListener listener = listenerRef.getAndSet(null);
                    if (listener != null) {
                        try {
                            listener.onError(executionError);
                        } catch (RemoteException e) {
                            Timber.e(e, e.getMessage());
                        }
                    }
                }

                @Override
                public void onTimeout() {
                    final ICommandListener listener = listenerRef.getAndSet(null);
                    if (listener != null) {
                        try {
                            listener.onTimeout();
                        } catch (RemoteException e) {
                            Timber.e(e, e.getMessage());
                        }
                    }
                }
            });
        }
    }

    public void sendAck(int step) {
        if (calibrating) {
            if(usingVehiclePos) {
                Log.v(TAG, "sendAck(): Send vehicle pos: " + step);
                MavLinkCalibration.sendVehiclePos(myDrone, step);
            } else {
                Log.v(TAG, "sendAck(): Send ack message");
                MavLinkCalibration.sendCalibrationAckMessage(myDrone, step);
            }
        }
    }

    public void processMessage(MAVLinkMessage msg) {
        if(!calibrating) return;

        switch(msg.msgid) {
            case msg_statustext.MAVLINK_MSG_ID_STATUSTEXT: {
                msg_statustext statusMsg = (msg_statustext) msg;
                final String message = statusMsg.getText();

                if (message != null && (message.startsWith("Place vehicle") || message.startsWith("Calibration"))) {
                    handler.post(onCalibrationStart);

                    usingVehiclePos = false;

                    mavMsg = message;
                    if (message.startsWith("Calibration"))
                        calibrating = false;

                    myDrone.notifyDroneEvent(DroneEventsType.CALIBRATION_IMU);
                }
                break;
            }

            case msg_command_long.MAVLINK_MSG_ID_COMMAND_LONG: {
                msg_command_long cmd = (msg_command_long)msg;

                switch(cmd.command) {
                    case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS: {
                        final int vehiclePos = Math.round(cmd.param1);
                        usingVehiclePos = true;

                        if(vehiclePos == currVehiclePos) return;

                        switch(vehiclePos) {
                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_LEVEL: {
                                mavMsg = "Place the vehicle level and press Next.";
                                break;
                            }

                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_LEFT: {
                                mavMsg = "Place the vehicle on its LEFT side and press Next.";
                                break;
                            }

                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_RIGHT: {
                                mavMsg = "Place the vehicle on its RIGHT side and press Next.";
                                break;
                            }

                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_NOSEDOWN: {
                                mavMsg = "Place the vehicle nose DOWN and press Next.";
                                break;
                            }

                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_NOSEUP: {
                                mavMsg = "Place the vehicle nose UP and press Next.";
                                break;
                            }

                            case ACCELCAL_VEHICLE_POS.ACCELCAL_VEHICLE_POS_BACK: {
                                mavMsg = "Place the vehicle on its BACK and press Next.";
                                break;
                            }

                            default: {
                                calibrating = false;
                                break;
                            }
                        }

                        myDrone.notifyDroneEvent(DroneEventsType.CALIBRATION_IMU);
                        currVehiclePos = vehiclePos;
                        break;
                    }
                }

                break;
            }
        }
    }

    public String getMessage() {
        return mavMsg;
    }

    public boolean isCalibrating() {
        return calibrating;
    }

    @Override
    public void onDroneEvent(DroneEventsType event, MavLinkDrone drone) {
        switch (event) {
            case HEARTBEAT_TIMEOUT:
            case DISCONNECTED:
                if (calibrating)
                    cancelCalibration();
                break;
        }
    }

    public void cancelCalibration() {
        mavMsg = "";
        calibrating = false;
    }
}
