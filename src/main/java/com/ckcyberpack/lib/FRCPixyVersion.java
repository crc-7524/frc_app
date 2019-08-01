package com.ckcyberpack.lib;

public class FRCPixyVersion {

    private int fwMajor;
    private int fwMinor;
    private int fwBuild;
    private String fwType;


    public FRCPixyVersion(int major, int minor, int build, String type){
        fwMajor = major;
        fwMinor = minor;
        fwBuild = build;
        fwType = type;
    }

    public String getFirmware(){
        return String.valueOf(fwMajor) + "." + String.valueOf(fwMinor) + String.valueOf(fwBuild);
    }

    public int getFirmwareMajor(){
        return fwMajor;
    }

    public int getFirmwareMinor(){
        return fwMinor;
    }

    public int getFirmwareBuild(){
        return fwBuild;
    }

    public String getFirmwareType(){
        return fwType;
    }

}
