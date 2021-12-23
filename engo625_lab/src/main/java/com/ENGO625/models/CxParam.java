package com.ENGO625.models;

public class CxParam {

	// Variance of UERE
	private double varUERE;
	// Diagonal elements of Cofactor matrix used to derive DOP parameters
	private double[] dopDiag;
	// Incase estimator used was Kalam Filter
	private double[] enuCov;
	private boolean isEnuCovPresent = false;

	public CxParam(double varUERE, double[] dopDiag) {
		super();
		this.varUERE = varUERE;
		this.dopDiag = dopDiag;
	}

	public CxParam(double[] enuCov) {
		this.enuCov = enuCov;
		isEnuCovPresent = true;
	}

	public double getVarUERE() {
		return varUERE;
	}

	public double[] getDopDiag() {
		return dopDiag;
	}

	public double[] getEnuCov() {
		return enuCov;
	}

	public boolean isEnuCovPresent() {
		return isEnuCovPresent;
	}

}
