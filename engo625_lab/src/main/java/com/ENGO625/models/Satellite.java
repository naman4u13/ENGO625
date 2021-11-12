package com.ENGO625.models;

public class Satellite {

	// GPS time
	private int t;
	// Satellite PRN
	private int prn;
	// ECEF(in m)
	private double[] ecef;
	// Vel(in m/s)
	private double[] vel;

	public Satellite(int t, int prn, double[] ecef, double[] vel) {
		super();
		this.t = t;
		this.prn = prn;
		this.ecef = ecef;
		this.vel = vel;
	}

	public Satellite(double[] data) {
		super();
		this.prn = (int) data[0];
		this.t = (int) data[1];
		this.ecef = new double[] { data[2], data[3], data[4] };
		this.vel = new double[] { data[5], data[6], data[7] };

	}

	public int getT() {
		return t;
	}

	public int getPrn() {
		return prn;
	}

	public double[] getEcef() {
		return ecef;
	}

	public double[] getVel() {
		return vel;
	}

}
