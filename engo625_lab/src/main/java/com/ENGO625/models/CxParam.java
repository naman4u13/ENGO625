package com.ENGO625.models;

public class CxParam {

	// Variance of UERE
	private double varUERE;
	// Diagonal elements of Cofactor matrix used to derive DOP parameters
	private double[] dopDiag;

	public CxParam(double varUERE, double[] dopDiag) {
		super();
		this.varUERE = varUERE;
		this.dopDiag = dopDiag;
	}

	public double getVarUERE() {
		return varUERE;
	}

	public double[] getDopDiag() {
		return dopDiag;
	}
}
