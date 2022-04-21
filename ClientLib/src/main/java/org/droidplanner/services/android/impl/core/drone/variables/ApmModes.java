package org.droidplanner.services.android.impl.core.drone.variables;

import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

import java.util.ArrayList;
import java.util.List;

public enum ApmModes implements BaseMode<ApmModes> {
	FIXED_WING_MANUAL (0,"Manual",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_CIRCLE (1,"Circle",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_STABILIZE (2,"Stabilize",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_TRAINING (3,"Training",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_ACRO(4, "Acro", MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_FLY_BY_WIRE_A (5,"FBW A",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_FLY_BY_WIRE_B (6,"FBW B",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_CRUISE(7, "Cruise", MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_AUTOTUNE(8, "AutoTune", MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_AUTO (10,"Auto",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_RTL (11,"RTL",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_LOITER (12,"Loiter",MAV_TYPE.MAV_TYPE_FIXED_WING),
	FIXED_WING_GUIDED (15,"Guided",MAV_TYPE.MAV_TYPE_FIXED_WING),

	ROTOR_STABILIZE(0, "Stabilize", MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_ACRO(1,"Acro", MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_ALT_HOLD(2, "Alt Hold",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_AUTO(3, "Auto",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_GUIDED(4, "Guided",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_LOITER(5, "Loiter",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_RTL(6, "RTL",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_CIRCLE(7, "Circle",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_LAND(9, "Land",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_TOY(11, "Drift",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_SPORT(13, "Sport",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_FLIP(14, "Flip",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_AUTOTUNE(15, "Autotune",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_POSHOLD(16, "PosHold",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_BRAKE(17,"Brake",MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_THROW(18, "Throw", MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_AVOID_ADSB(19,"Avoid ADSB", MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_GUIDED_NOGPS(20,"Guided NoGPS", MAV_TYPE.MAV_TYPE_QUADROTOR),
	ROTOR_SMART_RTL(21, "Smart RTL", MAV_TYPE.MAV_TYPE_QUADROTOR),

	ROVER_MANUAL(0, "MANUAL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_ACRO(1, "ACRO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_LEARNING(2, "LEARNING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_STEERING(3, "STEERING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_HOLD(4, "HOLD", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_LOITER(5, "LOITER", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_FOLLOW(6, "FOLLOW", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_SIMPLE(7, "SIMPLE", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_AUTO(10, "AUTO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_RTL(11, "RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_SMARTRTL(12, "SMART_RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_GUIDED(15, "GUIDED", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
	ROVER_INITIALIZING(16, "INITIALIZING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),

	VTOL_STABILIZE(17, "QSTABILIZE", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
	VTOL_HOVER(18, "QHOVER", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
	VTOL_LOITER(19, "QLOITER", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
	VTOL_LAND(20, "QLAND", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
	VTOL_RTL(21, "QRTL", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),

	UNKNOWN(-1, "Unknown", MAV_TYPE.MAV_TYPE_GENERIC);

	private final long number;
    private final String name;
	private final int type;

	ApmModes(long number,String name, int type){
		this.number = number;
		this.name = name;
		this.type = type;
	}

	public long getNumber() {
		return number;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public ApmModes getNativeMode() {
		return this;
	}

	public int getType() {
		return type;
	}

	public static ApmModes getMode(long i, int type) {
        if (isCopter(type)) {
            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        } else if (isVtol(type)) {
			type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR;
		}

		for (ApmModes mode : ApmModes.values()) {
			if (i == mode.getNumber() && type == mode.getType()) {
				return mode;
			}
		}
		return UNKNOWN;
	}

	public static ApmModes getMode(String str, int type) {
        if (isCopter(type)) {
            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        } else if (isVtol(type)) {
			type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR;
		}

		for (ApmModes mode : ApmModes.values()) {
			if (str.equals(mode.getName()) && type == mode.getType()) {
				return mode;
			}
		}
		return UNKNOWN;
	}

	public static List<ApmModes> getModeList(int type) {

		if (isCopter(type)) {
			type = MAV_TYPE.MAV_TYPE_QUADROTOR;
		} else if (isVtol(type)) {
			type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR;
		}

		List<ApmModes> modeList = new ArrayList<>();
		for (ApmModes mode : ApmModes.values()) {
			if (mode.getType() == type) {
				modeList.add(mode);
			}
		}
		return modeList;
	}

	public static List<VehicleMode> getUserModesForType(int type) {
		final List<VehicleMode> modes = new ArrayList<>();

		for(ApmModes m: getModeList(type)) {
			final VehicleMode vm = getVehicleMode(m);
			if(vm != null) {
				modes.add(vm);
			}
		}

		return modes;
	}

	public static boolean isValid(ApmModes mode) {
		return mode!=ApmModes.UNKNOWN;
	}

	public static boolean isCopter(int type){
		switch (type) {
		case MAV_TYPE.MAV_TYPE_TRICOPTER:
		case MAV_TYPE.MAV_TYPE_QUADROTOR:
		case MAV_TYPE.MAV_TYPE_HEXAROTOR:
		case MAV_TYPE.MAV_TYPE_OCTOROTOR:
		case MAV_TYPE.MAV_TYPE_HELICOPTER:
			return true;

		default:
			return false;
		}
	}

	public static boolean isVtol(int type) {
		return Type.isVtol(type);
	}

	public static VehicleMode getVehicleMode(ApmModes mode) {
		switch (mode) {
			case FIXED_WING_MANUAL:
				return VehicleMode.PLANE_MANUAL;
			case FIXED_WING_CIRCLE:
				return VehicleMode.PLANE_CIRCLE;

			case FIXED_WING_STABILIZE:
				return VehicleMode.PLANE_STABILIZE;

			case FIXED_WING_TRAINING:
				return VehicleMode.PLANE_TRAINING;

			case FIXED_WING_ACRO:
				return VehicleMode.PLANE_ACRO;

			case FIXED_WING_FLY_BY_WIRE_A:
				return VehicleMode.PLANE_FLY_BY_WIRE_A;

			case FIXED_WING_FLY_BY_WIRE_B:
				return VehicleMode.PLANE_FLY_BY_WIRE_B;

			case FIXED_WING_CRUISE:
				return VehicleMode.PLANE_CRUISE;

			case FIXED_WING_AUTOTUNE:
				return VehicleMode.PLANE_AUTOTUNE;

			case FIXED_WING_AUTO:
				return VehicleMode.PLANE_AUTO;

			case FIXED_WING_RTL:
				return VehicleMode.PLANE_RTL;

			case FIXED_WING_LOITER:
				return VehicleMode.PLANE_LOITER;

			case FIXED_WING_GUIDED:
				return VehicleMode.PLANE_GUIDED;

			case ROTOR_STABILIZE:
				return VehicleMode.COPTER_STABILIZE;

			case ROTOR_ACRO:
				return VehicleMode.COPTER_ACRO;

			case ROTOR_ALT_HOLD:
				return VehicleMode.COPTER_ALT_HOLD;

			case ROTOR_AUTO:
				return VehicleMode.COPTER_AUTO;

			case ROTOR_GUIDED:
				return VehicleMode.COPTER_GUIDED;

			case ROTOR_LOITER:
				return VehicleMode.COPTER_LOITER;

			case ROTOR_RTL:
				return VehicleMode.COPTER_RTL;

			case ROTOR_CIRCLE:
				return VehicleMode.COPTER_CIRCLE;

			case ROTOR_LAND:
				return VehicleMode.COPTER_LAND;

			case ROTOR_TOY:
				return VehicleMode.COPTER_DRIFT;

			case ROTOR_SPORT:
				return VehicleMode.COPTER_SPORT;

			case ROTOR_FLIP:
				return VehicleMode.COPTER_FLIP;

			case ROTOR_AUTOTUNE:
				return VehicleMode.COPTER_AUTOTUNE;

			case ROTOR_POSHOLD:
				return VehicleMode.COPTER_POSHOLD;

			case ROTOR_BRAKE:
				return VehicleMode.COPTER_BRAKE;

			case ROTOR_THROW:
				return VehicleMode.COPTER_THROW;

			case ROTOR_AVOID_ADSB:
				return VehicleMode.COPTER_AVOID_ADSB;

			case ROTOR_GUIDED_NOGPS:
				return VehicleMode.COPTER_GUIDED_NOGPS;

			case ROTOR_SMART_RTL:
				return VehicleMode.COPTER_SMART_RTL;

			case ROVER_MANUAL:
				return VehicleMode.ROVER_MANUAL;

			case ROVER_STEERING:
				return VehicleMode.ROVER_STEERING;

			case ROVER_HOLD:
				return VehicleMode.ROVER_HOLD;

			case ROVER_AUTO:
				return VehicleMode.ROVER_AUTO;

			case ROVER_RTL:
				return VehicleMode.ROVER_RTL;

			case ROVER_SMARTRTL:
				return VehicleMode.ROVER_SMART_RTL;

			case ROVER_ACRO:
				return VehicleMode.ROVER_ACRO;

			case ROVER_GUIDED:
				return VehicleMode.ROVER_GUIDED;

			case ROVER_INITIALIZING:
				return VehicleMode.ROVER_INITIALIZING;

			default:
			case UNKNOWN:
				return null;

		}
	}
}
