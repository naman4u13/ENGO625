package com.ENGO625.estimation.kalman_filter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.stream.IntStream;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.simple.SimpleMatrix;

import com.ENGO625.models.Observation;

public class EKF {

	private KFconfig kfObj;
	private double prObsNoiseVar;

	public EKF() {
		kfObj = new KFconfig();
	}

	public ArrayList<double[]> process(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList,
			double[] intialBaseline) throws Exception {

		double[][] x = new double[3][1];
		double[][] P = new double[3][3];
		/*
		 * state XYZ is intialized WLS generated Rx position estimated using first epoch
		 * data, a-priori estimate error covariance matrix for state XYZ is therefore
		 * assigned 4 m^2 value. Other state variables are assigned infinite(big)
		 * variance
		 */
		IntStream.range(0, 3).forEach(i -> x[i][0] = intialBaseline[i]);
		IntStream.range(0, 3).forEach(i -> P[i][i] = 4);

		kfObj.setState_ProcessCov(x, P);
		// Begin iteration or recursion
		return iterate(baselineObsList, timeList);

	}

	private ArrayList<double[]> iterate(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList)
			throws Exception {
		ArrayList<double[]> baselineList = new ArrayList<double[]>();
		HashMap<Integer, Integer> map = new HashMap<Integer, Integer>();
		for (int i = 0; i < timeList.size(); i++) {
			ArrayList<Observation> baselineObsvs = baselineObsList.get(i);
			int n = baselineObsvs.size();
			SimpleMatrix x = kfObj.getState();
			SimpleMatrix P = kfObj.getCovariance();
			SimpleMatrix _x = new SimpleMatrix(3 + n, 1);
			SimpleMatrix _P = new SimpleMatrix(3 + n, 3 + n);
			HashMap<Integer, Integer> newMap = new HashMap<Integer, Integer>();
			_x.insertIntoThis(0, 0, x.extractMatrix(0, 3, 0, 1));
			_P.insertIntoThis(0, 0, P.extractMatrix(0, 3, 0, 3));
			for (int j = 3 + 0; j < 3 + n; j++) {
				Observation obs = baselineObsvs.get(j - 3);
				int prn = obs.getPrn();
				if (map.containsKey(prn) && obs.isPhaseLocked() == true) {
					int k = map.get(prn);
					_x.set(j, x.get(k));
					_P.set(0, j, P.get(0, k));
					_P.set(1, j, P.get(1, k));
					_P.set(2, j, P.get(2, k));
					_P.set(j, 0, P.get(k, 0));
					_P.set(j, 1, P.get(k, 1));
					_P.set(j, 2, P.get(k, 2));
					_P.set(j, j, P.get(k, k));
					for (int l = j + 1; l < 3 + n; l++) {
						int _prn = baselineObsvs.get(l - 3).getPrn();
						if (map.containsKey(_prn)) {
							int m = map.get(_prn);
							_P.set(j, l, P.get(k, m));
							_P.set(l, j, P.get(m, k));

						}
					}

				} else {
					_x.set(j, obs.getPhaseL1() - obs.getPseudorange());
					_P.set(j, j, 1e8);
					if (obs.isPhaseLocked() == false) {
						for (int l = j - 1; l > 2; l--) {
							_P.set(j, l, 0);
							_P.set(l, j, 0);
						}
					}

				}

				newMap.put(prn, j);
			}
			kfObj.setState_ProcessCov(_x, _P);
			map.clear();
			map.putAll(newMap);
			// Perform Predict and Update
			runFilter(baselineObsvs);
			// Fetch Posteriori state estimate and estimate error covariance matrix
			x = kfObj.getState();
			P = kfObj.getCovariance();
			double[] estBaseline = new double[] { x.get(0), x.get(1), x.get(2) };
			// Add position estimate to the list
			baselineList.add(estBaseline);
			/*
			 * Check whether estimate error covariance matrix is positive semidefinite
			 * before further proceeding
			 */
			if (!MatrixFeatures_DDRM.isPositiveDefinite(P.getMatrix())) {

				throw new Exception("PositiveDefinite test Failed");
			}

		}

		return baselineList;
	}

	private void runFilter(ArrayList<Observation> baselineObsvs) {

		// Satellite count
		int n = baselineObsvs.size();

		// Assign Q and F matrix
		kfObj.config();
		kfObj.predict();

		SimpleMatrix x = kfObj.getState();
		// double[] estECEF = new double[] { x.get(0), x.get(1), x.get(2) };

		/*
		 * H is the Jacobian matrix of partial derivat.01 ives Observation StateModel(h)
		 * of with respect to x
		 */
		SimpleMatrix H = new SimpleMatrix(getJacobian(baselineObsvs));
		// Measurement vector
		double[][] z = new double[2 * n][1];
		IntStream.range(0, n).forEach(i -> z[i][0] = baselineObsvs.get(i).getPseudorange());
		IntStream.range(n, 2 * n).forEach(i -> z[i][0] = baselineObsvs.get(i - n).getPhaseL1());
		// Estimated Measurement vector
		SimpleMatrix ze = H.mult(x);

		// Measurement Noise
		double[][] R = new double[2 * n][2 * n];
		for (int i = 0; i < n; i++) {
			R[i][i] = 10;
			R[n + i][n + i] = 10e-4;
		}
		// Perform Update Step
		kfObj.update(z, R, ze, H);

	}

	private double[][] getJacobian(ArrayList<Observation> baselineObsvs) {
		int n = baselineObsvs.size();
		double[][] H = new double[2 * n][3 + n];

		for (int _i = 0; _i < n; _i++) {
			final int i = _i;
			Observation obs = baselineObsvs.get(i);
			IntStream.range(0, 3).forEach(j -> H[i][j] = -obs.getUnitLOS()[j]);
			IntStream.range(0, 3).forEach(j -> H[n + i][j] = -obs.getUnitLOS()[j]);
			H[n + i][3 + i] = 1;
		}

		return H;

	}
}
