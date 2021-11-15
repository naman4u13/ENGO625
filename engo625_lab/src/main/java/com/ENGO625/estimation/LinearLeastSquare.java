package com.ENGO625.estimation;

import java.util.ArrayList;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO625.models.CxParam;
import com.ENGO625.models.Observation;
import com.ENGO625.util.LatLonUtil;

public class LinearLeastSquare {
	private final static double SpeedofLight = 299792458;
	private SimpleMatrix HtWHinv = null;
	private double sigmaUERE = 1;
	private double[] estEcefClk;
	private double[] residual;

	public double[] process(ArrayList<Observation> obsList, boolean isWLS) throws Exception {

		// Satellite count
		int n = obsList.size();
		// Weight matrix
		double[][] weight = new double[n][n];
		/*
		 * If 'isWLS' flag is true, the estimation method is WLS and weight matrix will
		 * be based on elevation angle otherwise identity matrix will assigned for LS
		 */
		if (isWLS) {
			for (int i = 0; i < n; i++) {
				double elevAngle = obsList.get(i).getElevAngle();
				double var = 1 / Math.pow(Math.sin(elevAngle), 2);
				weight[i][i] = 1 / var;
			}

		} else {
			IntStream.range(0, n).forEach(i -> weight[i][i] = 1);
		}
		// variable to store estimated Rx position and clk offset
		estEcefClk = new double[4];
		/*
		 * Error variable based on norm value deltaX vector, intially assigned a big
		 * value
		 */
		double error = Double.MAX_VALUE;
		// Threshold to stop iteration or regression
		double threshold = 1e-3;
		// Minimum 4 satellite are required to proceed
		if (n >= 4) {

			while (error >= threshold) {

				// Misclosure vector
				double[][] deltaPR = new double[n][1];
				// Jacobian or Design Matrix
				double[][] h = new double[n][4];
				// Iterate through each satellite, to compute LOS vector and Approx pseudorange
				for (int i = 0; i < n; i++) {

					Observation obs = obsList.get(i);
					double[] satECEF = obs.getEcef();
					double PR = obs.getPseduorange();
					// Approx Geometric Range
					double approxGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - estEcefClk[j])
							.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
					// Approx Pseudorange Range
					double approxPR = approxGR + (SpeedofLight * estEcefClk[3]);
					deltaPR[i][0] = approxPR - PR;
					int index = i;
					IntStream.range(0, 3).forEach(j -> h[index][j] = (satECEF[j] - estEcefClk[j]) / approxGR);
					h[i][3] = 1;
				}
				// Least Squares implementation
				SimpleMatrix H = new SimpleMatrix(h);
				SimpleMatrix Ht = H.transpose();
				SimpleMatrix W = new SimpleMatrix(weight);
				HtWHinv = (Ht.mult(W).mult(H)).invert();
				SimpleMatrix DeltaPR = new SimpleMatrix(deltaPR);
				SimpleMatrix DeltaX = HtWHinv.mult(Ht).mult(W).mult(DeltaPR);
				// updating Rx state vector, by adding deltaX vector
				IntStream.range(0, 3).forEach(i -> estEcefClk[i] = estEcefClk[i] + DeltaX.get(i, 0));
				estEcefClk[3] += (-DeltaX.get(3, 0)) / SpeedofLight;
				// Recomputing error - norm of deltaX vector
				error = Math.sqrt(IntStream.range(0, 3).mapToDouble(i -> Math.pow(DeltaX.get(i, 0), 2)).reduce(0,
						(i, j) -> i + j));

			}
			double[][] res = new double[n][1];
			residual = new double[n];
			for (int i = 0; i < n; i++) {
				Observation obs = obsList.get(i);
				double PR = obs.getPseduorange();
				double PR_hat = Math
						.sqrt(IntStream.range(0, 3).mapToDouble(j -> obs.getEcef()[j] - estEcefClk[j])
								.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble())
						+ (SpeedofLight * estEcefClk[3]);
				res[i][0] = PR_hat - PR;
				residual[i] = res[i][0];
				System.out.print("");
			}
			SimpleMatrix W = new SimpleMatrix(weight);
			SimpleMatrix R = new SimpleMatrix(res);
			double estVarFact = (R.transpose().mult(W).mult(R).get(0)) / (n - 4);
			/*
			 * Regression is completed, error is below threshold, successfully estimated Rx
			 * Position and Clk Offset
			 */
			return estEcefClk;
		}

		throw new Exception("Satellite count is less than 4, can't compute user position");

	}

	public CxParam getCxParam() {
		SimpleMatrix R = getR(estEcefClk);
		SimpleMatrix dopEnu = R.mult(HtWHinv.extractMatrix(0, 3, 0, 3)).mult(R.transpose());
		double[] dopDiag = IntStream.range(0, 3).mapToDouble(i -> dopEnu.get(i, i)).toArray();

		return new CxParam(Math.pow(sigmaUERE, 2), dopDiag);
	}

	private SimpleMatrix getR(double[] ecef) {
		double[] llh = LatLonUtil.ecef2lla(ecef);
		double lat = Math.toRadians(llh[0]);
		double lon = Math.toRadians(llh[1]);
		double[][] r = new double[][] { { -Math.sin(lon), Math.cos(lon), 0 },
				{ -Math.sin(lat) * Math.cos(lon), -Math.sin(lat) * Math.sin(lon), Math.cos(lat) },
				{ Math.cos(lat) * Math.cos(lon), Math.cos(lat) * Math.sin(lon), Math.sin(lat) } };
		SimpleMatrix R = new SimpleMatrix(r);
		return R;
	}

	public double[] getResidual() {
		return residual;
	}

}
