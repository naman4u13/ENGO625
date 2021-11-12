package com.ENGO625.util;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.net.URL;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

import com.ENGO625.models.Observation;
import com.ENGO625.models.Satellite;

public class Parser {

	public static <T> HashMap<Integer, ArrayList<T>> getList(String fileName, Class<T> fileType) throws Exception {
		URL resource = Parser.class.getClassLoader().getResource(fileName);
		File file = Paths.get(resource.toURI()).toFile();
		DataInputStream in = new DataInputStream(new FileInputStream(file));
		boolean flag = true;
		int n = 6;
		if (fileType.getSimpleName() == Satellite.class.getSimpleName()) {
			flag = false;
			n = 8;
		}
		HashMap<Integer, ArrayList<T>> map = new HashMap<Integer, ArrayList<T>>();
		while (in.available() > 0) {
			ArrayList<T> list = new ArrayList<T>();
			int t = 0;
			for (int i = 0; i < 12; i++) {
				double[] data = new double[n];
				for (int j = 0; j < n; j++) {
					long l = in.readLong();
					data[j] = Double.longBitsToDouble(Long.reverseBytes(l));
				}

				if (data[0] > 0) {
					if (flag) {
						Observation obs = new Observation(data);
						list.add((T) obs);
					} else {
						Satellite sat = new Satellite(data);
						list.add((T) sat);
					}
					t = (int) data[1];
				}
			}
			map.put(t, list);
		}
		return map;
	}

	public static HashMap<Integer, HashMap<Integer, Satellite>> getSat(String fileName) throws Exception {
		URL resource = Parser.class.getClassLoader().getResource(fileName);
		File file = Paths.get(resource.toURI()).toFile();
		DataInputStream in = new DataInputStream(new FileInputStream(file));
		HashMap<Integer, HashMap<Integer, Satellite>> map = new HashMap<Integer, HashMap<Integer, Satellite>>();
		while (in.available() > 0) {
			for (int i = 0; i < 12; i++) {
				double[] data = new double[8];
				for (int j = 0; j < 8; j++) {
					long l = in.readLong();
					data[j] = Double.longBitsToDouble(Long.reverseBytes(l));
				}
				if (data[0] > 0) {
					int prn = (int) data[0];
					int t = (int) data[1];
					Satellite sat = new Satellite(data);
					map.computeIfAbsent(t, k -> new HashMap<Integer, Satellite>()).put(prn, sat);
				}
			}
		}
		return map;
	}

	public static HashMap<String, double[]> getTrueEcef(String fileName) throws Exception {
		URL resource = Parser.class.getClassLoader().getResource(fileName);
		File file = Paths.get(resource.toURI()).toFile();
		Scanner input = new Scanner(file);
		HashMap<String, double[]> trueEcef = new HashMap<String, double[]>();
		while (input.hasNextLine()) {
			String[] rcvr = input.nextLine().split("\\s+");
			String key = rcvr[0].replace(':', ' ').strip();
			double[] lla = new double[3];
			for (int i = 0; i < 2; i++) {
				int j = (i * 3) + 1;
				double deg = Double.parseDouble(rcvr[j]);
				double min = Double.parseDouble(rcvr[j + 1]);
				double sec = Double.parseDouble(rcvr[j + 2]);
				int sign = deg >= 0 ? deg > 0 ? 1 : 0 : -1;
				lla[i] = sign * (Math.abs(deg) + (min / 60) + (sec / 3600));
			}
			lla[2] = Double.parseDouble(rcvr[7]);
			double[] ecef = LatLonUtil.lla2ecef(lla);
			trueEcef.put(key, ecef);
		}

		return trueEcef;
	}

}
