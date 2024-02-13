package util;

/**
 * A helper class which defines some useful functions.
 */
public final class Util {

	private Util() {
		System.out.println("A Util class cannot be instantiated!");
	}

	/**
     * Converts an array of Strings to a list of doubles
     */
	public static double[] convertDouble(String[] stringList){
		int strLen = stringList.length;
		double[] doubleList = new double[strLen];
		for (int i = 0; i < strLen; i++) {
			doubleList[i] = Double.parseDouble(stringList[i]);
		}
		return doubleList;
	}
	
	/**
	 * Returns the bin ID for a specific theta given the number of bins.
	 * 
	 * @param theta		the angle in radian to convert to the corresponding bin number
	 * @param numBins	the number of bins comprising 360 degrees
	 */
	public static int thetaToBinId(double theta, int numBins){
		int binId = (int) (( (theta / (2 * Math.PI)) * numBins) % numBins);
		if (binId < 0) return binId + numBins;
		else return binId;
	}
}
