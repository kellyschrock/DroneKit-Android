package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;

import com.o3dr.services.android.lib.coordinate.LatLongAlt;

public class TargetPosition implements DroneAttribute {

    private final LatLongAlt coordinate = new LatLongAlt(0, 0,  0);
    private double yaw = 0;
    private long lastUpdate = 0L;

    public TargetPosition() {
        super();
    }

    public TargetPosition(double lat, double lng, double alt, double yaw, long lastUpdate) {
        this();
        coordinate.setLatitude(lat);
        coordinate.setLongitude(lng);
        coordinate.setAltitude(alt);
        this.yaw = yaw;
        this.lastUpdate = lastUpdate;
    }

    public LatLongAlt getCoordinate() { return coordinate; }
    public double getYaw() { return yaw; }
    public long getLastUpdate() { return lastUpdate; }

    public void update(double lat, double lng, double alt, double yaw, long when) {
        coordinate.setLatitude(lat);
        coordinate.setLongitude(lng);
        coordinate.setAltitude(alt);
        this.lastUpdate = when;
    }

    public boolean isEqualTo(double lat, double lng, double alt, double yaw) {
        return coordinate.getLatitude() == lat &&
            coordinate.getLongitude() == lng &&
            coordinate.getAltitude() == alt &&
            this.yaw == yaw;
    }

    public void clear() {
        update(0, 0, 0, 0, 0);
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeDouble(coordinate.getLatitude());
        dest.writeDouble(coordinate.getLongitude());
        dest.writeDouble(coordinate.getAltitude());
        dest.writeDouble(this.yaw);
        dest.writeLong(this.lastUpdate);
    }

    private TargetPosition(Parcel in) {
        this.coordinate.setLatitude(in.readDouble());
        this.coordinate.setLongitude(in.readDouble());
        this.coordinate.setAltitude(in.readDouble());
        this.yaw = in.readDouble();
        this.lastUpdate = in.readLong();
    }

    public static final Creator<TargetPosition> CREATOR = new Creator<TargetPosition>() {
        public TargetPosition createFromParcel(Parcel source) {
            return new TargetPosition(source);
        }

        public TargetPosition[] newArray(int size) {
            return new TargetPosition[size];
        }
    };
}

