package com.ENGO625.estimation.kalman_filter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.stream.IntStream;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.simple.SimpleMatrix;

import com.ENGO625.models.Observation;

public class EKF {

	private static final double wavelengthL1 = 0.1902936727983649;
	private KFconfig kfObj;
	private double prObsNoiseVar;
	private HashMap<String, HashMap<String, double[]>> ambInfoMap;

	public EKF() {
		kfObj = new KFconfig();
		ambInfoMap = new HashMap<String, HashMap<String, double[]>>();
	}

	public ArrayList<double[]> process(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList,
			double[] intialBaseline, SimpleMatrix R) throws Exception {

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
		return iterate2(baselineObsList, timeList, R);

	}

	private ArrayList<double[]> iterate(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList,
			SimpleMatrix R) throws Exception {
		ArrayList<double[]> baselineList = new ArrayList<double[]>();
		HashMap<Integer, Integer> map = new HashMap<Integer, Integer>();
		int len = timeList.size();
//		ambInfoMap.put("e", new double[len]);
//		ambInfoMap.put("n", new double[len]);
//		ambInfoMap.put("u", new double[len]);
		for (int i = 0; i < len; i++) {
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
						Observation _obs = baselineObsvs.get(l - 3);
						int _prn = _obs.getPrn();
						if (map.containsKey(_prn) && _obs.isPhaseLocked() == true) {
							int m = map.get(_prn);
							_P.set(j, l, P.get(k, m));
							_P.set(l, j, P.get(m, k));

						}
					}

				} else {
					_x.set(j, (obs.getPhaseL1() - obs.getPseudorange()) / wavelengthL1);
					_P.set(j, j, 1e8);

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
//			SimpleMatrix enuCov = R.mult(P.extractMatrix(0, 3, 0, 3)).mult(R.transpose());
//			ambInfoMap.get("e")[i] = Math.sqrt(enuCov.get(0, 0));
//			ambInfoMap.get("n")[i] = Math.sqrt(enuCov.get(1, 1));
//			ambInfoMap.get("u")[i] = Math.sqrt(enuCov.get(2, 2));

			for (int prn : map.keySet()) {
				int j = map.get(prn);
				String PRN = Integer.toString(prn);
				ambInfoMap.computeIfAbsent(PRN, k -> new HashMap<String, double[]>())
						.computeIfAbsent("Ambiguity Float Value", k -> new double[len])[i] = x.get(j);
				ambInfoMap.get(PRN).computeIfAbsent("Ambiguity Standard Deviation", k -> new double[len])[i] = Math
						.sqrt(P.get(j, j));
				ambInfoMap.get(PRN).computeIfAbsent(
						"Phase Lock(0 - Signal Lost, 1 - Lock Lost/Cycle Slip, 2 - Phase Locked)",
						k -> new double[len])[i] = baselineObsvs.get(j - 3).isPhaseLocked() ? 2 : 1;

			}

		}

		return baselineList;
	}

	private ArrayList<double[]> iterate2(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList,
			SimpleMatrix R) throws Exception {
		int t_thresh = 1000;
		ArrayList<double[]> baselineList = new ArrayList<double[]>();
		HashMap<Integer, Integer> map = new HashMap<Integer, Integer>();
		int len = timeList.size();
		int n0 = baselineObsList.get(0).size();
		int[] intAmb = new int[2 * n0];
		double[] sosr = new double[(int) Math.pow(2, n0)];
		boolean isFixed = false;

		for (int i = 0; i < len; i++) {
			ArrayList<Observation> baselineObsvs = baselineObsList.get(i);
			Collections.sort(baselineObsvs, (o1, o2) -> o1.getPrn() - o2.getPrn());
			int n = baselineObsvs.size();
			if (n != n0) {
				System.err.print("ERROR");
				throw new Exception("ERROR");
			}
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
						Observation _obs = baselineObsvs.get(l - 3);
						int _prn = _obs.getPrn();
						if (map.containsKey(_prn) && _obs.isPhaseLocked() == true) {
							int m = map.get(_prn);
							_P.set(j, l, P.get(k, m));
							_P.set(l, j, P.get(m, k));

						}
					}

				} else {
					_x.set(j, (obs.getPhaseL1() - obs.getPseudorange()) / wavelengthL1);
					_P.set(j, j, 1e8);

				}

				newMap.put(prn, j);
			}

			map.clear();
			map.putAll(newMap);
			if (i >= t_thresh && !isFixed) {

				if (i == t_thresh) {
					System.out.println("AT t = " + i);
					for (int j = 0; j < n; j++) {
						System.out.println(baselineObsvs.get(j).getPrn() + " amb. std dev: "
								+ Math.sqrt(_P.get(j + 3, j + 3)) + "  float ambiguity: " + _x.get(j + 3));

						double floatAmb = _x.get(j + 3);
						intAmb[2 * j] = (int) Math.floor(floatAmb);
						intAmb[(2 * j) + 1] = (int) Math.ceil(floatAmb);
					}
				}
				double[] ambRes = new double[2 * n];
				for (int j = 0; j < n; j++) {
					ambRes[2 * j] = _x.get(j + 3) - intAmb[2 * j];
					ambRes[(2 * j) + 1] = _x.get(j + 3) - intAmb[(2 * j) + 1];
				}
				SimpleMatrix Cn = _P.extractMatrix(3, 3 + n, 3, 3 + n);

				updateSOSR(ambRes, new double[n][1], sosr, 0, 0, getAmbWeight(Cn), n0);

				int setNo = ratioTest(sosr);
				if (setNo > -1) {
					double[] _fixedIntAmb = getFixedIntAmb(setNo, intAmb);
					SimpleMatrix ambFixedInt = new SimpleMatrix(n, 1, true, _fixedIntAmb);
					SimpleMatrix CxFloat = _P.extractMatrix(0, 3, 0, 3);
					SimpleMatrix Cxn = _P.extractMatrix(0, 3, 3, 3 + n);
					SimpleMatrix xFloat = _x.extractMatrix(0, 3, 0, 1);
					SimpleMatrix ambFloat = _x.extractMatrix(3, 3 + n, 0, 1);

					SimpleMatrix xFixed = xFloat.minus(Cxn.mult(Cn.invert()).mult(ambFloat.minus(ambFixedInt)));
					SimpleMatrix CxFixed = CxFloat.minus(Cxn.mult(Cn.invert()).mult(Cxn.transpose()));
					_x = new SimpleMatrix(xFixed.concatRows(ambFixedInt));
					_P = new SimpleMatrix(3 + n, 3 + n);
					_P.insertIntoThis(0, 0, CxFixed);
					isFixed = true;

				}
			}
			kfObj.setState_ProcessCov(_x, _P);
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

			if (!MatrixFeatures_DDRM.isPositiveSemidefinite(P.getMatrix())) {

				throw new Exception("PositiveSemiDefinite test Failed");
			}
			for (int prn : map.keySet()) {
				int j = map.get(prn);
				String PRN = Integer.toString(prn);
				ambInfoMap.computeIfAbsent(PRN, k -> new HashMap<String, double[]>())
						.computeIfAbsent("Ambiguity Float Value", k -> new double[len])[i] = x.get(j);
				ambInfoMap.get(PRN).computeIfAbsent("Ambiguity Standard Deviation", k -> new double[len])[i] = Math
						.sqrt(P.get(j, j));
				ambInfoMap.get(PRN).computeIfAbsent(
						"Phase Lock(0 - Signal Lost, 1 - Lock Lost/Cycle Slip, 2 - Phase Locked)",
						k -> new double[len])[i] = baselineObsvs.get(j - 3).isPhaseLocked() ? 2 : 1;

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
			H[n + i][3 + i] = wavelengthL1;
		}

		return H;

	}

	public HashMap<String, HashMap<String, double[]>> getAmbInfoMap() {
		return ambInfoMap;
	}

	private void updateSOSR(double[] ambRes, double[][] res, double[] sosr, int i, int j, SimpleMatrix W, int n0) {
		if (i < n0) {
			res[i][0] = ambRes[2 * i];
			updateSOSR(ambRes, res, sosr, i + 1, (2 * j) + 1, W, n0);
			res[i][0] = ambRes[(2 * i) + 1];
			updateSOSR(ambRes, res, sosr, i + 1, (2 * j) + 2, W, n0);
		} else {
			int j0 = (int) Math.pow(2, n0) - 1;
			SimpleMatrix r = new SimpleMatrix(res);
			sosr[j - j0] = sosr[j - j0] + r.transpose().mult(W).mult(r).get(0);
		}

	}

	private int ratioTest(double[] sosr) {

		double threshold = 3;
		int n = sosr.length;
		double min = Double.MAX_VALUE;
		double min2 = Double.MAX_VALUE;
		int index = -1;
		for (int i = 0; i < n; i++) {
			if (sosr[i] <= min) {
				min2 = min;
				min = sosr[i];
				index = i;
			}
		}
		if ((min2 / min) > threshold) {
			return index;
		}
		return -1;
	}

	private double[] getFixedIntAmb(int index, int[] intAmb) {
		int n = intAmb.length / 2;
		index += (int) Math.pow(2, n) - 1;
		double[] fixedIntAmb = new double[n];
		for (int i = n - 1; i >= 0; i--) {

			if (index % 2 == 0) {
				index = (index - 2) / 2;
				fixedIntAmb[i] = intAmb[(2 * i) + 1];
			} else {
				index = (index - 1) / 2;
				fixedIntAmb[i] = intAmb[2 * i];
			}

		}
		return fixedIntAmb;
	}

	private SimpleMatrix getAmbWeight(SimpleMatrix C) {
		SimpleMatrix W = C.invert();
		SimpleMatrix diag = W.diag();
		double min = Double.MAX_VALUE;
		int n = W.numRows();
		for (int i = 0; i < n; i++) {
			min = Math.min(min, diag.get(i));
		}
		W = W.divide(min);
		return W;

	}
}
