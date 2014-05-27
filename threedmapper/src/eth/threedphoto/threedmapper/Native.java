/*
 * Author: Bharat Tak
 */

package eth.threedphoto.threedmapper;

public class Native {

	static {
		System.loadLibrary("opencv_java");
		System.loadLibrary("octomath_user");
	}

	public native int[] map(int i, int j, String imagepath );
}
