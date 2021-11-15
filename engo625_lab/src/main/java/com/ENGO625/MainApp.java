package com.ENGO625;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.json.JSONObject;

import com.ENGO625.estimation.LinearLeastSquare;
import com.ENGO625.models.CxParam;
import com.ENGO625.models.Observation;
import com.ENGO625.models.SatResidual;
import com.ENGO625.models.Satellite;
import com.ENGO625.util.ComputeEleAzm;
import com.ENGO625.util.GraphPlotter;
import com.ENGO625.util.LatLonUtil;
import com.ENGO625.util.Parser;

public class MainApp {

	public static void main(String args[]) {

		try {
			// Path to store output file
			String path = "D:\\projects\\eclipse_projects\\UCalgary\\ENGO625\\results\\output3";
			File output = new File(path + ".txt");
			PrintStream stream;
			stream = new PrintStream(output);
			System.setOut(stream);
			HashMap<String, double[]> trueEcef = Parser.getTrueEcef("TruthCoordinates.txt");
			HashMap<Integer, HashMap<Integer, Satellite>> satMap = Parser.getSat("Satellites.sat");
			HashMap<Integer, ArrayList<Observation>> baseMap = Parser.getList("BaseL1L2.obs", Observation.class);
			HashMap<Integer, ArrayList<Observation>> remoteMap = Parser.getList("RemoteL1L2.obs", Observation.class);
			// Map based variable to store enu errors for each algorithm
			HashMap<String, ArrayList<double[]>> EnuMap = new HashMap<String, ArrayList<double[]>>();
			HashMap<String, ArrayList<CxParam>> CxMap = new HashMap<String, ArrayList<CxParam>>();
			ArrayList<Integer> satCountList = new ArrayList<Integer>();
			HashMap<String, HashMap<Integer, ArrayList<SatResidual>>> satResMap = new HashMap<String, HashMap<Integer, ArrayList<SatResidual>>>();
			ArrayList<Integer> timeList = new ArrayList<Integer>();
			int opt = 3;
			if (opt == 1) {
				JSONObject json = new JSONObject(satMap);
				FileWriter file = new FileWriter(
						"D:\\projects\\eclipse_projects\\UCalgary\\ENGO625\\results\\satMap2.json");
				file.write(json.toString());
				file.close();
			}

			TreeMap<Integer, ArrayList<Observation>> remoteOrdMap = new TreeMap<Integer, ArrayList<Observation>>();
			// Copy all data from hashMap into TreeMap
			remoteOrdMap.putAll(remoteMap);
			double[] remoteEcef = trueEcef.get("Remote");
			double[] baseEcef = trueEcef.get("Base");
			int t0 = remoteOrdMap.firstKey();
			for (Map.Entry ele : remoteOrdMap.entrySet()) {
				int t = (int) ele.getKey();
				timeList.add(t - t0);
				HashMap<Integer, Satellite> subSatMap = satMap.get(t);
				ArrayList<Observation> obsList = (ArrayList<Observation>) ele.getValue();
				ArrayList<Observation> invalid = new ArrayList<Observation>();
				for (Observation obs : obsList) {
					int prn = obs.getPrn();
					if (!subSatMap.containsKey(prn)) {
						invalid.add(obs);
						continue;
					}
					Satellite sat = subSatMap.get(prn);
					obs.setEcef(sat.getEcef());
					obs.setVel(sat.getVel());
				}
				obsList.removeAll(invalid);
				int n = obsList.size();
				satCountList.add(n);
				switch (opt) {

				case 2, 3, 4:

					LinearLeastSquare lls = new LinearLeastSquare();
					double[] ecef = lls.process(obsList, false);
					for (int i = 0; i < n; i++) {
						double[] elevAzm = ComputeEleAzm.computeEleAzm(ecef, obsList.get(i).getEcef());
						obsList.get(i).setElevAzmAngle(elevAzm);
					}
					if (opt == 2 || opt == 4) {
						EnuMap.computeIfAbsent("LS", k -> new ArrayList<double[]>()).add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("LS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("LS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = obsList.get(i);
							satResMap.get("LS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}
					}
					if (opt == 3 || opt == 4) {

						lls = new LinearLeastSquare();
						ecef = lls.process(obsList, true);
						EnuMap.computeIfAbsent("WLS", k -> new ArrayList<double[]>())
								.add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("WLS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("WLS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = obsList.get(i);
							satResMap.get("WLS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}

					}
					break;
				}
			}

			// Calculate Accuracy Metrics
			HashMap<String, ArrayList<double[]>> GraphEnuMap = new HashMap<String, ArrayList<double[]>>();
			for (String key : EnuMap.keySet()) {
				ArrayList<Double>[] errList = new ArrayList[5];
				IntStream.range(0, 5).forEach(i -> errList[i] = new ArrayList<Double>());
				ArrayList<double[]> enuList = EnuMap.get(key);
				int n = enuList.size();

				for (int i = 0; i < n; i++) {
					double[] enu = enuList.get(i);
					// error in East direction
					errList[0].add(Math.sqrt(enu[0] * enu[0]));
					// error in North direction
					errList[1].add(Math.sqrt(enu[1] * enu[1]));
					// error in Up direction
					errList[2].add(Math.sqrt(enu[2] * enu[2]));
					// 3d error
					errList[3].add(Math.sqrt(Arrays.stream(enu).map(j -> j * j).sum()));
					// 2d error
					errList[4].add(Math.sqrt((enu[0] * enu[0]) + (enu[1] * enu[1])));

				}

				GraphEnuMap.put(key, enuList);

				// RMSE
				System.out.println("\n" + key + " RMSE");

				System.out.println(" E - " + RMS(errList[0]));
				System.out.println(" N - " + RMS(errList[1]));
				System.out.println(" U - " + RMS(errList[2]));
				System.out.println(" 3d Error - " + RMS(errList[3]));
				System.out.println(" 2d Error - " + RMS(errList[4]));

				// 95th Percentile
				IntStream.range(0, 5).forEach(i -> Collections.sort(errList[i]));
				int q95 = (int) (n * 0.95);

				System.out.println("\n" + key + " 95%");

				System.out.println(" E - " + errList[0].get(q95));
				System.out.println(" N - " + errList[1].get(q95));
				System.out.println(" U - " + errList[2].get(q95));
				System.out.println(" 3d Error - " + errList[3].get(q95));
				System.out.println(" 2d Error - " + errList[4].get(q95));

			}

			// Plot Error Graphs
			// GraphPlotter.graphENU(GraphEnuMap, CxMap, timeList);
			// GraphPlotter.graphDOP(CxMap, satCountList, timeList);
			GraphPlotter.graphSatRes(satResMap);

		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// Transform from ECEF to ENU
	public static double[] estimateENU(double[] estEcefClk, double[] rxECEF) {
		double[] enu = LatLonUtil.ecef2enu(estEcefClk, rxECEF);
		return enu;
	}

	public static double RMS(ArrayList<Double> list) {
		return Math.sqrt(list.stream().mapToDouble(x -> x * x).average().orElse(Double.NaN));
	}

	public static double MAE(ArrayList<Double> list) {
		return list.stream().mapToDouble(x -> x).average().orElse(Double.NaN);
	}

}
