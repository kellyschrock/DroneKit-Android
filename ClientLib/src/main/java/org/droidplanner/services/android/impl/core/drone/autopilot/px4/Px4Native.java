package org.droidplanner.services.android.impl.core.drone.autopilot.px4;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.util.Log;

import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.ardupilotmega.msg_camera_feedback;
import com.MAVLink.ardupilotmega.msg_mag_cal_progress;
import com.MAVLink.ardupilotmega.msg_mag_cal_report;
import com.MAVLink.ardupilotmega.msg_mount_configure;
import com.MAVLink.ardupilotmega.msg_mount_status;
import com.MAVLink.ardupilotmega.msg_radio;
import com.MAVLink.common.msg_named_value_int;
import com.MAVLink.common.msg_raw_imu;
import com.MAVLink.common.msg_rc_channels_raw;
import com.MAVLink.common.msg_servo_output_raw;
import com.MAVLink.common.msg_statustext;
import com.MAVLink.common.msg_vfr_hud;
import com.MAVLink.enums.MAV_MOUNT_MODE;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.action.ControlActions;
import com.o3dr.services.android.lib.drone.action.ExperimentalActions;
import com.o3dr.services.android.lib.drone.action.GimbalActions;
import com.o3dr.services.android.lib.drone.action.ParameterActions;
import com.o3dr.services.android.lib.drone.action.StateActions;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError;
import com.o3dr.services.android.lib.drone.mission.action.MissionActions;
import com.o3dr.services.android.lib.drone.property.DroneAttribute;
import com.o3dr.services.android.lib.drone.property.Parameter;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.gcs.action.CalibrationActions;
import com.o3dr.services.android.lib.model.AbstractCommandListener;
import com.o3dr.services.android.lib.model.ICommandListener;
import com.o3dr.services.android.lib.model.action.Action;

import org.droidplanner.services.android.impl.communication.model.DataLink;
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands;
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkParameters;
import org.droidplanner.services.android.impl.core.MAVLink.WaypointManager;
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds;
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces;
import org.droidplanner.services.android.impl.core.drone.LogMessageListener;
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.APMConstants;
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone;
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes;
import org.droidplanner.services.android.impl.core.drone.variables.Camera;
import org.droidplanner.services.android.impl.core.drone.variables.GuidedPoint;
import org.droidplanner.services.android.impl.core.drone.variables.Magnetometer;
import org.droidplanner.services.android.impl.core.drone.variables.RC;
import org.droidplanner.services.android.impl.core.drone.variables.calibration.AccelCalibration;
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl;
import org.droidplanner.services.android.impl.core.firmware.FirmwareType;
import org.droidplanner.services.android.impl.core.mission.Mission;
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser;
import org.droidplanner.services.android.impl.utils.CommonApiUtils;

import timber.log.Timber;

/**
 * Created by Fredia Huya-Kouadio on 9/10/15.
 */
public class Px4Native extends GenericMavLinkDrone {

    private final org.droidplanner.services.android.impl.core.drone.variables.RC rc;
    private final Mission mission;
    private final GuidedPoint guidedPoint;
    private final AccelCalibration accelCalibrationSetup;
    private final WaypointManager waypointManager;
    private final Magnetometer mag;
    private final Camera camera;
    private final MagnetometerCalibrationImpl magCalibration;

    public Px4Native(String droneId, Context context, Handler handler, DataLink.DataLinkProvider<MAVLinkMessage> mavClient, AutopilotWarningParser warningParser, LogMessageListener logListener) {
        super(droneId, context, handler, mavClient, warningParser, logListener);

        this.rc = new RC(this);
        this.mission = new Mission(this);
        this.waypointManager = new WaypointManager(this, handler);
        this.guidedPoint = new GuidedPoint(this, handler);
        this.accelCalibrationSetup = new AccelCalibration(this, handler);
        this.magCalibration = new MagnetometerCalibrationImpl(this);
        this.mag = new Magnetometer(this);
        this.camera = new Camera(this);
    }

    @Override
    public FirmwareType getFirmwareType() {
        return FirmwareType.PX4_NATIVE;
    }

    @Override
    public boolean executeAsyncAction(Action action, final ICommandListener listener) {
        final String type = action.getType();
        final Bundle data = action.getData();

        switch(type) {
            case MissionActions.ACTION_LOAD_WAYPOINTS:
                CommonApiUtils.loadWaypoints(this);
                return true;

            case MissionActions.ACTION_SET_MISSION:
                data.setClassLoader(com.o3dr.services.android.lib.drone.mission.Mission.class.getClassLoader());
                com.o3dr.services.android.lib.drone.mission.Mission mission = data.getParcelable(MissionActions.EXTRA_MISSION);
                boolean pushToDrone = data.getBoolean(MissionActions.EXTRA_PUSH_TO_DRONE);
                CommonApiUtils.setPX4Mission(this, mission, pushToDrone);
                return true;

            case MissionActions.ACTION_START_MISSION:
                boolean forceModeChange = data.getBoolean(MissionActions.EXTRA_FORCE_MODE_CHANGE);
                boolean forceArm = data.getBoolean(MissionActions.EXTRA_FORCE_ARM);
                CommonApiUtils.startMission(this, forceModeChange, forceArm, listener);
                return true;

            // EXPERIMENTAL ACTIONS
            case ExperimentalActions.ACTION_EPM_COMMAND:
                boolean release = data.getBoolean(ExperimentalActions.EXTRA_EPM_RELEASE);
                CommonApiUtils.epmCommand(this, release, listener);
                return true;

            case ExperimentalActions.ACTION_TRIGGER_CAMERA:
                CommonApiUtils.triggerCamera(this);
                return true;

            case ExperimentalActions.ACTION_SET_ROI:
                LatLongAlt roi = data.getParcelable(ExperimentalActions.EXTRA_SET_ROI_LAT_LONG_ALT);
                if (roi != null) {
                    MavLinkDoCmds.setROI(this, roi, listener);
                }
                return true;

            case ExperimentalActions.ACTION_SET_RELAY:
                int relayNumber = data.getInt(ExperimentalActions.EXTRA_RELAY_NUMBER);
                boolean isOn = data.getBoolean(ExperimentalActions.EXTRA_IS_RELAY_ON);
                MavLinkDoCmds.setRelay(this, relayNumber, isOn, listener);
                return true;

            case ExperimentalActions.ACTION_SET_SERVO:
                int channel = data.getInt(ExperimentalActions.EXTRA_SERVO_CHANNEL);
                int pwm = data.getInt(ExperimentalActions.EXTRA_SERVO_PWM);
                MavLinkDoCmds.setServo(this, channel, pwm, listener);
                return true;

            // CONTROL ACTIONS
            case ControlActions.ACTION_SEND_GUIDED_POINT: {
                data.setClassLoader(LatLongAlt.class.getClassLoader());
                boolean force = data.getBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT);
                LatLongAlt guidedPoint = data.getParcelable(ControlActions.EXTRA_GUIDED_POINT);
                Timber.d("ACTION_SEND_GUIDED_POINT: guidedPoint=%s force=%s", guidedPoint, force);

                CommonApiUtils.sendGuidedPoint(this, guidedPoint, force, listener);
                return true;
            }

            case ControlActions.ACTION_SEND_GUIDED_POINT_DIRECT: {
                data.setClassLoader(LatLongAlt.class.getClassLoader());
                LatLongAlt point = data.getParcelable(ControlActions.EXTRA_GUIDED_POINT);
                Timber.d("ACTION_SEND_GUIDED_POINT_DIRECT: point=%s", point);

                MavLinkCommands.sendGuidedPosition(this,
                        point.getLatitude(),
                        point.getLongitude(),
                        point.getAltitude());
                return true;
            }

            case ControlActions.ACTION_LOOK_AT_TARGET:
                boolean force = data.getBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT);
                LatLongAlt lookAtTarget = data.getParcelable(ControlActions.EXTRA_LOOK_AT_TARGET);
                CommonApiUtils.sendLookAtTarget(this, lookAtTarget, force, listener);
                return true;

            case ControlActions.ACTION_RESET_ROI:
                CommonApiUtils.sendResetROI(this, listener);
                return true;

            case ControlActions.ACTION_SET_GUIDED_ALTITUDE:
                double guidedAltitude = data.getDouble(ControlActions.EXTRA_ALTITUDE);
                Timber.d("ACTION_SET_GUIDED_ALTITUDE: alt=%.1f", guidedAltitude);

                CommonApiUtils.setGuidedAltitude(this, guidedAltitude);
                return true;

            // PARAMETER ACTIONS
            case ParameterActions.ACTION_REFRESH_PARAMETERS:
                CommonApiUtils.refreshParameters(this);
                return true;

            case ParameterActions.ACTION_WRITE_PARAMETERS:
                data.setClassLoader(com.o3dr.services.android.lib.drone.property.Parameters.class.getClassLoader());
                com.o3dr.services.android.lib.drone.property.Parameters parameters = data.getParcelable(ParameterActions.EXTRA_PARAMETERS);
                CommonApiUtils.writeParameters(this, parameters);
                if(!updateParametersFrom(parameters)) {
                    Timber.w("Unable to update from passed parameters");
                }
                return true;

            // DRONE STATE ACTIONS
            case StateActions.ACTION_SET_VEHICLE_HOME:
                LatLongAlt homeLoc = data.getParcelable(StateActions.EXTRA_VEHICLE_HOME_LOCATION);
                if (homeLoc != null) {
                    MavLinkDoCmds.setVehicleHome(this, homeLoc, new AbstractCommandListener() {
                        @Override
                        public void onSuccess() {
                            CommonApiUtils.postSuccessEvent(listener);
                            requestHomeUpdate();
                        }

                        @Override
                        public void onError(int executionError) {
                            CommonApiUtils.postErrorEvent(executionError, listener);
                            requestHomeUpdate();
                        }

                        @Override
                        public void onTimeout() {
                            CommonApiUtils.postTimeoutEvent(listener);
                            requestHomeUpdate();
                        }
                    });
                } else {
                    CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener);
                }
                return true;

            //CALIBRATION ACTIONS
            case CalibrationActions.ACTION_START_IMU_CALIBRATION:
                CommonApiUtils.startIMUCalibration(this, listener);
                return true;

            case CalibrationActions.ACTION_SEND_IMU_CALIBRATION_ACK:
                int imuAck = data.getInt(CalibrationActions.EXTRA_IMU_STEP);
                CommonApiUtils.sendIMUCalibrationAck(this, imuAck);
                return true;

            case CalibrationActions.ACTION_START_MAGNETOMETER_CALIBRATION:
                boolean retryOnFailure = data.getBoolean(CalibrationActions.EXTRA_RETRY_ON_FAILURE, false);
                boolean saveAutomatically = data.getBoolean(CalibrationActions.EXTRA_SAVE_AUTOMATICALLY, true);
                int startDelay = data.getInt(CalibrationActions.EXTRA_START_DELAY, 0);
                CommonApiUtils.startMagnetometerCalibration(this, retryOnFailure, saveAutomatically, startDelay);
                return true;

            case CalibrationActions.ACTION_CANCEL_MAGNETOMETER_CALIBRATION:
                CommonApiUtils.cancelMagnetometerCalibration(this);
                return true;

            case CalibrationActions.ACTION_ACCEPT_MAGNETOMETER_CALIBRATION:
                CommonApiUtils.acceptMagnetometerCalibration(this);
                return true;

            //************ Gimbal ACTIONS *************//
            case GimbalActions.ACTION_SET_GIMBAL_ORIENTATION:
                float pitch = data.getFloat(GimbalActions.GIMBAL_PITCH);
                float roll = data.getFloat(GimbalActions.GIMBAL_ROLL);
                float yaw = data.getFloat(GimbalActions.GIMBAL_YAW);
                MavLinkDoCmds.setGimbalOrientation(this, pitch, roll, yaw, listener);
                return true;

            case GimbalActions.ACTION_RESET_GIMBAL_MOUNT_MODE:
            case GimbalActions.ACTION_SET_GIMBAL_MOUNT_MODE:
                if(data != null) {
                    int mountMode = data.getInt(GimbalActions.GIMBAL_MOUNT_MODE, MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING);
                    Timber.i("Setting gimbal mount mode: %d", mountMode);

                    Parameter mountParam = getParameterManager().getParameter("MNT_MODE");
                    if (mountParam == null) {
                        msg_mount_configure msg = new msg_mount_configure();
                        msg.target_system = getSysid();
                        msg.target_component = getCompid();
                        msg.mount_mode = (byte) mountMode;
                        msg.stab_pitch = 0;
                        msg.stab_roll = 0;
                        msg.stab_yaw = 0;
                        getMavClient().sendMessage(msg, listener);
                    } else {
                        MavLinkParameters.sendParameter(this, "MNT_MODE", 1, mountMode);
                    }
                }
                return true;

            default:
                return super.executeAsyncAction(action, listener);
        }
    }

    @Override
    public void onMavLinkMessageReceived(MAVLinkMessage message) {

        if(message == null) {
            // TODO: Should put filtering in for making sure this is meant for this vehicle.
            return;
        }

        if(!getParameterManager().processMessage(message)) {
            getWaypointManager().processMessage(message);
            getCalibrationSetup().processMessage(message);

            switch(message.msgid) {
                case msg_statustext.MAVLINK_MSG_ID_STATUSTEXT:
                    // These are any warnings sent from APM:Copter with
                    // gcs_send_text_P()
                    // This includes important thing like arm fails, prearm fails, low
                    // battery, etc.
                    // also less important things like "erasing logs" and
                    // "calibrating barometer"
                    msg_statustext msg_statustext = (msg_statustext) message;
                    processStatusText(msg_statustext);
                    break;

                case msg_vfr_hud.MAVLINK_MSG_ID_VFR_HUD:
                    processVfrHud((msg_vfr_hud) message);
                    break;

                case msg_raw_imu.MAVLINK_MSG_ID_RAW_IMU:
                    msg_raw_imu msg_imu = (msg_raw_imu) message;
                    mag.newData(msg_imu);
                    break;

                case msg_radio.MAVLINK_MSG_ID_RADIO:
                    msg_radio m_radio = (msg_radio) message;
                    processSignalUpdate(m_radio.rxerrors, m_radio.fixed, m_radio.rssi,
                            m_radio.remrssi, m_radio.txbuf, m_radio.noise, m_radio.remnoise);
                    break;

                case msg_rc_channels_raw.MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    rc.setRcInputValues((msg_rc_channels_raw) message);
                    break;

                case msg_servo_output_raw.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                    rc.setRcOutputValues((msg_servo_output_raw) message);
                    break;

                case msg_camera_feedback.MAVLINK_MSG_ID_CAMERA_FEEDBACK:
                    getCamera().newImageLocation((msg_camera_feedback) message);
                    break;

                case msg_mount_status.MAVLINK_MSG_ID_MOUNT_STATUS:
                    processMountStatus((msg_mount_status) message);
                    break;

                case msg_named_value_int.MAVLINK_MSG_ID_NAMED_VALUE_INT:
                    processNamedValueInt((msg_named_value_int) message);
                    break;

                //*************** Magnetometer calibration messages handling *************//
                case msg_mag_cal_progress.MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
                case msg_mag_cal_report.MAVLINK_MSG_ID_MAG_CAL_REPORT:
                    getMagnetometerCalibration().processCalibrationMessage(message);
                    break;

            }

            super.onMavLinkMessageReceived(message);
        }
    }

    @Override
    public Mission getMission() {
        return this.mission;
    }

    @Override
    public Camera getCamera() {
        return this.camera;
    }

    @Override
    public DroneAttribute getAttribute(String attributeType) {
        if (!TextUtils.isEmpty(attributeType)) {
            switch (attributeType) {

                case AttributeType.MISSION:
                    return CommonApiUtils.getMission(this);

                case AttributeType.GUIDED_STATE:
                    return CommonApiUtils.getGuidedState(this);

                case AttributeType.MAGNETOMETER_CALIBRATION_STATUS:
                    return CommonApiUtils.getMagnetometerCalibrationStatus(this);
            }
        }

        return super.getAttribute(attributeType);
    }

    @Override
    public GuidedPoint getGuidedPoint() {
        return this.guidedPoint;
    }

    @Override
    public AccelCalibration getCalibrationSetup() {
        return this.accelCalibrationSetup;
    }

    @Override
    public WaypointManager getWaypointManager() {
        return this.waypointManager;
    }

    @Override
    public MagnetometerCalibrationImpl getMagnetometerCalibration() {
        return this.magCalibration;
    }

    @Override
    protected boolean setVehicleMode(Bundle data, ICommandListener listener) {

        if(!super.setVehicleMode(data, listener)) {
            data.setClassLoader(VehicleMode.class.getClassLoader());
            VehicleMode newMode = data.getParcelable(StateActions.EXTRA_VEHICLE_MODE);
            CommonApiUtils.changePx4VehicleMode(this, newMode, listener);
        }

        return super.setVehicleMode(data, listener);
    }

    protected void processMountStatus(msg_mount_status mountStatus) {
        camera.updateMountOrientation(mountStatus);

        Bundle eventInfo = new Bundle(3);
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_PITCH, mountStatus.pointing_a / 100f);
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_ROLL, mountStatus.pointing_b / 100f);
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_YAW, mountStatus.pointing_c / 100f);
        notifyAttributeListener(AttributeEvent.GIMBAL_ORIENTATION_UPDATED, eventInfo);
    }

    protected void processStatusText(msg_statustext statusText) {
        String message = statusText.getText();
        if (TextUtils.isEmpty(message))
            return;

        Timber.d("processStatusText(): message=%s", message);

        // TODO: WRONG. Find out how to set FW version on PX4.
        if (message.startsWith("ArduCopter") || message.startsWith("ArduPlane")
                || message.startsWith("ArduRover") || message.startsWith("Solo")
                || message.startsWith("APM:Copter") || message.startsWith("APM:Plane")
                || message.startsWith("APM:Rover")) {
            setFirmwareVersion(message);
        } else {

            //Try parsing as an error.
            if (!getState().parseAutopilotError(message)) {

                //Relay to the connected client.
                int logLevel;
                switch (statusText.severity) {
                    case APMConstants.Severity.SEVERITY_CRITICAL:
                        logLevel = Log.ERROR;
                        break;

                    case APMConstants.Severity.SEVERITY_HIGH:
                        logLevel = Log.WARN;
                        break;

                    case APMConstants.Severity.SEVERITY_MEDIUM:
                        logLevel = Log.INFO;
                        break;

                    default:
                    case APMConstants.Severity.SEVERITY_LOW:
                        logLevel = Log.VERBOSE;
                        break;

                    case APMConstants.Severity.SEVERITY_USER_RESPONSE:
                        logLevel = Log.DEBUG;
                        break;
                }

                logMessage(logLevel, message);
            }
        }
    }

    protected void processVfrHud(msg_vfr_hud vfrHud) {
        if (vfrHud == null)
            return;

        setAltitudeGroundAndAirSpeeds(vfrHud.alt, vfrHud.groundspeed, vfrHud.airspeed, vfrHud.climb);
    }

    protected void setAltitudeGroundAndAirSpeeds(double altitude, double groundSpeed, double airSpeed, double climb) {
//        if (this.altitude.getAltitude() != altitude) {
//            this.altitude.setAltitude(altitude);
//            notifyDroneEvent(DroneInterfaces.DroneEventsType.ALTITUDE);
//        }

        if (speed.getGroundSpeed() != groundSpeed || speed.getAirSpeed() != airSpeed || speed.getVerticalSpeed() != climb) {
            speed.setGroundSpeed(groundSpeed);
            speed.setAirSpeed(airSpeed);
            speed.setVerticalSpeed(climb);

            notifyDroneEvent(DroneInterfaces.DroneEventsType.SPEED);
        }
    }

    protected void setAltitudes(double altitude) {
        if (this.altitude.getAltitude() != altitude) {
            this.altitude.setAltitude(altitude);
            notifyDroneEvent(DroneInterfaces.DroneEventsType.ALTITUDE);
        }
    }

    private void processNamedValueInt(msg_named_value_int message) {
        if (message == null)
            return;

        switch (message.getName()) {
            case "ARMMASK":
                //Give information about the vehicle's ability to arm successfully.
                ApmModes vehicleMode = getState().getMode();
                if (ApmModes.isCopter(vehicleMode.getType())) {
                    int value = message.value;
                    boolean isReadyToArm = (value & (1 << vehicleMode.getNumber())) != 0;
                    String armReadinessMsg = isReadyToArm ? "READY TO ARM" : "UNREADY FOR ARMING";
                    logMessage(Log.INFO, armReadinessMsg);
                }
                break;
        }
    }
}
