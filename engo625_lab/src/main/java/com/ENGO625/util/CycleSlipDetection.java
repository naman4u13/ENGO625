package com.ENGO625.util;

import java.util.ArrayList;
import java.util.HashMap;

import com.ENGO625.models.Observation;

public class CycleSlipDetection {
	private static final double freqL1 = 1575.42e6;
	// Speed of Light
	private static final double SPEED_OF_LIGHT = 299792458;
	// GPS L1 wavelength
	private static final double wavelengthL1 = SPEED_OF_LIGHT / freqL1;

	public static void phaseRateMethod(ArrayList<ArrayList<Observation>> baseObsList,
			ArrayList<ArrayList<Observation>> remObsList) {

		double threshold = wavelengthL1;
		int n = baseObsList.size();
		HashMap<Integer, Integer> order = new HashMap<Integer, Integer>();
		for (int i = 0; i < baseObsList.get(0).size(); i++) {
			order.put(baseObsList.get(0).get(i).getPrn(), i);
		}
		for (int i = 1; i < n; i++) {
			HashMap<Integer, Integer> newOrder = new HashMap<Integer, Integer>();
			for (int j = 0; j < baseObsList.get(i).size(); j++) {
				int prn = baseObsList.get(i).get(j).getPrn();

				if (order.containsKey(prn)) {
					int k = order.get(prn);
					CSdetect(remObsList, i, j, k, threshold);
					CSdetect(baseObsList, i, j, k, threshold);
				}
				newOrder.put(prn, j);
			}
			order.clear();
			order.putAll(newOrder);

		}
	}

	private static void CSdetect(ArrayList<ArrayList<Observation>> ObsList, int i, int j, int k, double threshold) {

		Observation currObs = ObsList.get(i).get(j);
		Observation prevObs = ObsList.get(i - 1).get(k);
		double predPhase = prevObs.getPhaseL1() + ((currObs.getRangeRate() + prevObs.getRangeRate()) / 2);
		if (Math.abs(predPhase - currObs.getPhaseL1()) < wavelengthL1) {
			currObs.setPhaseLocked(true);
		}

	}

}
