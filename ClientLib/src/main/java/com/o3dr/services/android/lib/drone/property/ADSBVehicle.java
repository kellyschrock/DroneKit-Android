package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;

import com.MAVLink.common.msg_adsb_vehicle;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;

/**
 * An ADSB vehicle
 */
public class ADSBVehicle implements DroneAttribute {

    public static ADSBVehicle populate(ADSBVehicle v, msg_adsb_vehicle msg) {
        v.icaoAddress = msg.ICAO_address;

        double newLat = msg.lat / 1E7;
        double newLong = msg.lon / 1E7;
        double altMeters = (msg.altitude / 1000); // mm->m
        v.coord = new LatLongAlt(newLat, newLong, altMeters);
        v.heading = msg.heading;
        v.horizVelocity = msg.hor_velocity / 100; // cm/s -> m/s
        v.vertVelocity = msg.ver_velocity / 100; // cm/s -> m/s

        v.flags = msg.flags;
        v.squawk = msg.squawk;
        v.altitudeType = msg.altitude_type;
        v.callSign = msg.getCallsign();
        v.emitterType = msg.emitter_type;
        v.tslc = msg.tslc;

        return v;
    }

    private long icaoAddress;
    private LatLongAlt coord;
    private double heading;
    private double horizVelocity; // meters/second
    private double vertVelocity; // meters/second
    private int squawk;
    private int flags; // ADSB_FLAGS
    private int altitudeType; // ADSB_ALTITUDE_TYPE
    private int emitterType; // ADSB_EMITTER_TYPE
    private int tslc;
    private String callSign;

    public ADSBVehicle() {
        super();
    }

    public long getIcaoAddress() {
        return icaoAddress;
    }

    public LatLongAlt getCoord() {
        return coord;
    }

    public double getHeading() {
        return heading;
    }

    public double getHorizVelocity() {
        return horizVelocity;
    }

    public double getVertVelocity() {
        return vertVelocity;
    }

    public int getSquawk() {
        return squawk;
    }

    public int getFlags() {
        return flags;
    }

    public int getAltitudeType() {
        return altitudeType;
    }

    public int getEmitterType() {
        return emitterType;
    }

    public int getTslc() {
        return tslc;
    }

    public String getCallSign() {
        return callSign;
    }

    @Override
    public String toString() {
        return "ADSBVehicle{" +
                "icaoAddress=" + icaoAddress +
                ", coord=" + coord +
                ", heading=" + heading +
                ", horizVelocity=" + horizVelocity +
                ", vertVelocity=" + vertVelocity +
                ", squawk=" + squawk +
                ", flags=" + flags +
                ", altitudeType=" + altitudeType +
                ", emitterType=" + emitterType +
                ", tslc=" + tslc +
                ", callSign='" + callSign + '\'' +
                '}';
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeLong(icaoAddress);
        dest.writeParcelable(coord, 0);
        dest.writeDouble(heading);
        dest.writeDouble(horizVelocity);
        dest.writeDouble(vertVelocity);
        dest.writeInt(squawk);
        dest.writeInt(flags);
        dest.writeInt(altitudeType);
        dest.writeInt(emitterType);
        dest.writeInt(tslc);
        dest.writeString(callSign);
    }

    private ADSBVehicle(Parcel in) {
        icaoAddress = in.readLong();
        coord = in.readParcelable(LatLongAlt.class.getClassLoader());
        heading = in.readDouble();
        horizVelocity = in.readDouble();
        vertVelocity = in.readDouble();
        squawk = in.readInt();
        flags = in.readInt();
        altitudeType = in.readInt();
        emitterType = in.readInt();
        tslc = in.readInt();
        callSign = in.readString();
    }

    public static final Creator<ADSBVehicle> CREATOR = new Creator<ADSBVehicle>() {
        public ADSBVehicle createFromParcel(Parcel source) {
            return new ADSBVehicle(source);
        }

        public ADSBVehicle[] newArray(int size) {
            return new ADSBVehicle[size];
        }
    };
}
