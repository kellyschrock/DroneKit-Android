package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;

public class SolexCCState implements DroneAttribute {

    public static final int SOLEXCC_PORT = 80;

    private String ipAddress;

    public SolexCCState(){}

    public SolexCCState(String ipAddress) {
        super();
        this.ipAddress = ipAddress;
    }

    public void setTo(SolexCCState other) {
        this.ipAddress = other.ipAddress;
    }

    public String getIpAddress() { return ipAddress; }
    public void setIpAddress(String a) { ipAddress = a; }

    public boolean isValid() {
        return (ipAddress != null);
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeString(this.ipAddress);
    }

    private SolexCCState(Parcel in) {
        this.ipAddress = in.readString();
    }

    public static final Creator<SolexCCState> CREATOR = new Creator<SolexCCState>() {
        public SolexCCState createFromParcel(Parcel source) {
            return new SolexCCState(source);
        }

        public SolexCCState[] newArray(int size) {
            return new SolexCCState[size];
        }
    };
}
