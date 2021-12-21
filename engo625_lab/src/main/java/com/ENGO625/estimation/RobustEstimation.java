package com.ENGO625.estimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO625.models.Observation;

public class RobustEstimation {
	private final static double SpeedofLight = 299792458;
	// Cofactor Matrix used for DOP param computation

	public static double[] processSP(ArrayList<Observation> obsList, double[] estEcefClk, IF_Weight IFWeight)
			throws Exception {

		double cm = 1.4815;

		// Satellite count
		int n = obsList.size();
		// Threshold to stop iteration or regression
		double threshold = 1e-3;
		// Minimum 4 satellite are required to proceed
		boolean isConverged = false;

		if (n >= 4) {
			int N = 20;
			while (!isConverged && N > 0) {
				N--;
				// residual
				double[] residual = new double[n];
				for (int i = 0; i < n; i++) {

					Observation obs = obsList.get(i);
					double[] satECEF = obs.getEcef();
					double PR = obs.getPseduorange();

					final double[] ecefClk = estEcefClk;
					// Approx Geometric Range
					double approxGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - ecefClk[j])
							.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
					// Approx Pseudorange Range
					double approxPR = approxGR + (SpeedofLight * estEcefClk[3]);
					residual[i] = PR - approxPR;

				}
				double medRes = getMedian(residual);
				double scale = cm * getMedian(Arrays.stream(residual).map(i -> Math.abs(i - medRes)).toArray());
				double[][] weight = new double[n][n];
				if (IFWeight == IF_Weight.HUBER) {
					double a = 1.5;
					IntStream.range(0, n).forEach(i -> weight[i][i] = Math.min(1, a / Math.abs(residual[i] / scale)));
				} else if (IFWeight == IF_Weight.HAMPEL) {
					double a = 1, b = 1.5, c = 3.0;
					for (int i = 0; i < n; i++) {
						double normRes = Math.abs(residual[i] / scale);
						if (normRes < a) {
							weight[i][i] = 1;
						} else if (normRes < b) {
							weight[i][i] = a / normRes;
						} else if (normRes < c) {
							weight[i][i] = a * (c - normRes) / ((c - b) * normRes);
						} else {
							weight[i][i] = 1e-7;
						}

					}

				} else if (IFWeight == IF_Weight.ANDREW) {
					double c = 2.1;
					for (int i = 0; i < n; i++) {
						double normRes = Math.abs(residual[i] / scale);
						if (normRes < 0.5 * Math.PI * c) {
							weight[i][i] = 1;
						} else if (normRes < Math.PI * c) {
							weight[i][i] = Math.sin(normRes / c);
						} else {
							weight[i][i] = 1e-7;
						}

					}
				}
				for (int i = 0; i < n; i++) {

					if (weight[i][i] <= 0) {
						System.err.println("NEGATIVE WEIGHT");
					}
				}

				/*
				 * Error variable based on norm value deltaX vector, intially assigned a big
				 * value
				 */
				double error = Double.MAX_VALUE;
				double[] _estEcefClk = Arrays.copyOf(estEcefClk, 4);

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
						double approxGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - _estEcefClk[j])
								.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
						// Approx Pseudorange Range
						double approxPR = approxGR + (SpeedofLight * _estEcefClk[3]);
						deltaPR[i][0] = approxPR - PR;
						int index = i;
						IntStream.range(0, 3).forEach(j -> h[index][j] = (satECEF[j] - _estEcefClk[j]) / approxGR);
						h[i][3] = 1;
					}
					// Least Squares implementation
					SimpleMatrix H = new SimpleMatrix(h);
					SimpleMatrix Ht = H.transpose();
					SimpleMatrix W = new SimpleMatrix(weight);
					SimpleMatrix HtWHinv = (Ht.mult(W).mult(H)).invert();
					SimpleMatrix DeltaPR = new SimpleMatrix(deltaPR);
					SimpleMatrix DeltaX = HtWHinv.mult(Ht).mult(W).mult(DeltaPR);
					// updating Rx state vector, by adding deltaX vector
					IntStream.range(0, 3).forEach(i -> _estEcefClk[i] = _estEcefClk[i] + DeltaX.get(i, 0));
					_estEcefClk[3] += (-DeltaX.get(3, 0)) / SpeedofLight;
					// Recomputing error - norm of deltaX vector
					error = Math.sqrt(IntStream.range(0, 3).mapToDouble(i -> Math.pow(DeltaX.get(i, 0), 2)).reduce(0,
							(i, j) -> i + j));

				}
				final double[] ecefClk = estEcefClk;
				double delta = Math.sqrt(
						IntStream.range(0, 3).mapToDouble(i -> ecefClk[i] - _estEcefClk[i]).map(i -> i * i).sum());
				estEcefClk = _estEcefClk;

				if (delta < threshold) {
					isConverged = true;
				}

			}
			/*
			 * Regression is completed, error is below threshold, successfully estimated Rx
			 * Position and Clk Offset
			 */
			return estEcefClk;
		}

		throw new Exception("Satellite count is less than 4, can't compute user position");

	}

	public static double[] processBRSD(ArrayList<Observation> baseObsvs, ArrayList<Observation> remObsvs,
			double[] trueBaseEcef, double[] estEcefClk, IF_Weight IFWeight) throws Exception {

		// Satellite count
		int n = remObsvs.size();
		ArrayList<Observation> obsList = new ArrayList<Observation>();
		for (int i = 0; i < n; i++) {
			Observation remObs = remObsvs.get(i);
			Observation baseObs = baseObsvs.get(i);
			if (remObs.getPrn() != baseObs.getPrn()) {
				throw new Exception("Invalid Base and Remote observation list");
			}
			double[] satECEF = remObs.getEcef();
			// Remote Station Pseudorange
			double remPR = remObs.getPseduorange();
			// Base Station Pseudorange
			double basePR = baseObs.getPseduorange();
			// Base Station Geometric Range
			double baseGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - trueBaseEcef[j])
					.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
			double corrPR = remPR - (basePR - baseGR);
			Observation obs = new Observation(remObs.getT(), remObs.getPrn(), corrPR, remObs.getPhaseL1(),
					remObs.getPhaseL2(), remObs.getDoppler());
			obs.setEcef(satECEF);
			obsList.add(obs);
		}

		double cm = 1.4815;

		// Threshold to stop iteration or regression
		double threshold = 1e-3;
		// Minimum 4 satellite are required to proceed
		boolean isConverged = false;

		if (n >= 4) {
			int N = 20;
			while (!isConverged && N > 0) {
				N--;
				// residual
				double[] residual = new double[n];
				for (int i = 0; i < n; i++) {

					Observation obs = obsList.get(i);
					double[] satECEF = obs.getEcef();
					double PR = obs.getPseduorange();

					final double[] ecefClk = estEcefClk;
					// Approx Geometric Range
					double approxGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - ecefClk[j])
							.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
					// Approx Pseudorange Range
					double approxPR = approxGR + (SpeedofLight * estEcefClk[3]);
					residual[i] = PR - approxPR;

				}
				double medRes = getMedian(residual);
				double scale = cm * getMedian(Arrays.stream(residual).map(i -> Math.abs(i - medRes)).toArray());
				double[][] weight = new double[n][n];
				if (IFWeight == IF_Weight.HUBER) {
					double a = 1.5;
					IntStream.range(0, n).forEach(i -> weight[i][i] = Math.min(1, a / Math.abs(residual[i] / scale)));
				} else if (IFWeight == IF_Weight.HAMPEL) {
					double a = 1.5, b = 3.5, c = 8.0;
					for (int i = 0; i < n; i++) {
						double normRes = Math.abs(residual[i] / scale);
						if (normRes < a) {
							weight[i][i] = 1;
						} else if (normRes < b) {
							weight[i][i] = a / normRes;
						} else if (normRes < c) {
							weight[i][i] = a * (c - normRes) / ((c - b) * normRes);
						} else {
							weight[i][i] = 1e-7;
						}

					}

				} else if (IFWeight == IF_Weight.ANDREW) {
					double c = 2.1;
					for (int i = 0; i < n; i++) {
						double normRes = Math.abs(residual[i] / scale);
						if (normRes < 0.5 * Math.PI * c) {
							weight[i][i] = 1;
						} else if (normRes < Math.PI * c) {
							weight[i][i] = Math.sin(normRes / c);
						} else {
							weight[i][i] = 1e-7;
						}

					}
				}
				for (int i = 0; i < n; i++) {

					if (weight[i][i] <= 0) {
						System.err.println("NEGATIVE WEIGHT");
					}
				}

				/*
				 * Error variable based on norm value deltaX vector, intially assigned a big
				 * value
				 */
				double error = Double.MAX_VALUE;
				double[] _estEcefClk = Arrays.copyOf(estEcefClk, 4);

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
						double approxGR = Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> satECEF[j] - _estEcefClk[j])
								.map(j -> Math.pow(j, 2)).reduce((j, k) -> j + k).getAsDouble());
						// Approx Pseudorange Range
						double approxPR = approxGR + (SpeedofLight * _estEcefClk[3]);
						deltaPR[i][0] = approxPR - PR;
						int index = i;
						IntStream.range(0, 3).forEach(j -> h[index][j] = (satECEF[j] - _estEcefClk[j]) / approxGR);
						h[i][3] = 1;
					}
					// Least Squares implementation
					SimpleMatrix H = new SimpleMatrix(h);
					SimpleMatrix Ht = H.transpose();
					SimpleMatrix W = new SimpleMatrix(weight);
					SimpleMatrix HtWHinv = (Ht.mult(W).mult(H)).invert();
					SimpleMatrix DeltaPR = new SimpleMatrix(deltaPR);
					SimpleMatrix DeltaX = HtWHinv.mult(Ht).mult(W).mult(DeltaPR);
					// updating Rx state vector, by adding deltaX vector
					IntStream.range(0, 3).forEach(i -> _estEcefClk[i] = _estEcefClk[i] + DeltaX.get(i, 0));
					_estEcefClk[3] += (-DeltaX.get(3, 0)) / SpeedofLight;
					// Recomputing error - norm of deltaX vector
					error = Math.sqrt(IntStream.range(0, 3).mapToDouble(i -> Math.pow(DeltaX.get(i, 0), 2)).reduce(0,
							(i, j) -> i + j));

				}
				final double[] ecefClk = estEcefClk;
				double delta = Math.sqrt(
						IntStream.range(0, 3).mapToDouble(i -> ecefClk[i] - _estEcefClk[i]).map(i -> i * i).sum());
				estEcefClk = _estEcefClk;

				if (delta < threshold) {
					isConverged = true;
				}

			}
			/*
			 * Regression is completed, error is below threshold, successfully estimated Rx
			 * Position and Clk Offset
			 */
			return estEcefClk;
		}

		throw new Exception("Satellite count is less than 4, can't compute user position");

	}

	// Function for calculating median
	private static double getMedian(double a[]) {
		int n = a.length;
		// First we sort the array
		Arrays.sort(a);

		// check for even case
		if (n % 2 != 0)
			return a[n / 2];

		return (a[(n - 1) / 2] + a[n / 2]) / 2.0;
	}

}
