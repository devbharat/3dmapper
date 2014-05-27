/*
 * Author: Bharat Tak
 */

package eth.threedphoto.threedmapper;


import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


public class MainActivity extends Activity {

	private String imagePathText = "/sdcard/Notes/dataset/dataset3/";
	private String posePathText = "/sdcard/Notes/dataset/dataset3/";
	int startImageNumber = 280;
	int stopImageNumber = 380;
	static int bunchImages = 10;
	private int imageCounter = 0;
	private int runtime = 0;
	private double total_runtime=0.0;
	private float fps=0;
	private double average_fps=0.0;
	private int res=0; 
	
	private TextView mainText;
	private TextView secondText;
	private TextView thirdText;
	private TextView forthText;
	
	private Handler mHandler;
	private Handler errHandler;
	
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);



		//Load shared libraries
		System.loadLibrary("gnustl_shared");		

		File f = new File(getCacheDir() + "/liboctomath.so.1.6");
		    if (!f.exists()) try {

		    InputStream is = getAssets().open("liboctomath.so.1.6");
		    int size = is.available();
		    byte[] buffer = new byte[size];
		    is.read(buffer);
		    is.close();


		    FileOutputStream fos = new FileOutputStream(f);
		    fos.write(buffer);
		    fos.close();
		  } catch (Exception e) { throw new RuntimeException(e); }

		//Load shared library from asset
		System.load(f.getAbsolutePath());	


		File ff = new File(getCacheDir() + "/liboctomap.so.1.6");
		    if (!ff.exists()) try {

		    InputStream isf = getAssets().open("liboctomap.so.1.6");
		    int sizef = isf.available();
		    byte[] bufferf = new byte[sizef];
		    isf.read(bufferf);
		    isf.close();


		    FileOutputStream fosf = new FileOutputStream(ff);
		    fosf.write(bufferf);
		    fosf.close();
		  } catch (Exception ef) { throw new RuntimeException(ef); }

		//Load shared library from asset  
		System.load(ff.getAbsolutePath());	


		mainText = (TextView) findViewById(R.id.mainText);
		secondText = (TextView) findViewById(R.id.textView1);
		thirdText = (TextView) findViewById(R.id.textView3);
		forthText = (TextView) findViewById(R.id.textView4);
		

		//Get edited image path
		final EditText imagepath = (EditText) findViewById(R.id.pathImage);
		final Button editImagePath = (Button) findViewById(R.id.changeImagePath);;
		editImagePath.setOnClickListener(new View.OnClickListener()
		{
		public void onClick(View v)
			{
                        imagePathText=imagepath.getText().toString();
			Toast msg = Toast.makeText(getBaseContext(),"Modified image path to " + imagePathText, Toast.LENGTH_SHORT);
			msg.show();
                        }
                });


		//Get edited pose file path
		final EditText posepath = (EditText) findViewById(R.id.pathPose);
		final Button editPosePath = (Button) findViewById(R.id.changePosePath);;
		editPosePath.setOnClickListener(new View.OnClickListener()
		{
		public void onClick(View v)
			{
                        posePathText=posepath.getText().toString();
			Toast msg = Toast.makeText(getBaseContext(),"Modified pose path to " + posePathText, Toast.LENGTH_SHORT);
			msg.show();
                        }
                });


		//Get edited start image number
		final EditText startimage = (EditText) findViewById(R.id.startImage);
		final Button editStartImage = (Button) findViewById(R.id.changeStartImage);;
		editStartImage.setOnClickListener(new View.OnClickListener()
		{
		public void onClick(View v)
			{
                        String startImageText=startimage.getText().toString();
			Toast msg = Toast.makeText(getBaseContext(), "Modified start image to " + startImageText, Toast.LENGTH_SHORT);
			startImageNumber = Integer.parseInt(startImageText);
			msg.show();
                        }
                });


		//get edited end image number
		final EditText stopimage = (EditText) findViewById(R.id.stopImage);
		final Button editStopImage = (Button) findViewById(R.id.changeStopImage);;
		editStopImage.setOnClickListener(new View.OnClickListener()
		{
		public void onClick(View v)
			{
                        String stopImageText=stopimage.getText().toString();
			Toast msg = Toast.makeText(getBaseContext(),"Modified end image to " + stopImageText, Toast.LENGTH_SHORT);
			stopImageNumber = Integer.parseInt(stopImageText);
			msg.show();
                        }
                });


		//output analysis data
		final TextView secondText = (TextView) findViewById(R.id.textView1);
		final TextView thirdText = (TextView) findViewById(R.id.textView3);
		final TextView forthText = (TextView) findViewById(R.id.textView4);

			
		//Main: Execute JNI code on button click (Click only once! May launch multiple threads on multiple clicks!)
		final Button button = (Button) findViewById(R.id.button_send);
		button.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				mHandler = new Handler();
				Thread background = new Thread(new Runnable() {
					int i = startImageNumber;
					
					public void run() {
					while(i<=stopImageNumber){
						int k[];

						//Bad start end combination
						if (startImageNumber > stopImageNumber){
							errHandler.post(raise_error);
							break;
				

						//Bunches of images processed and added to previous graph file together
						}else if ((i + bunchImages) <= stopImageNumber){
							k = new Native().map(i,i + bunchImages,imagePathText);
							i = i + bunchImages;

							
							fps = (float)k[1]*1000/(k[0]);
							runtime = k[0];
							total_runtime = total_runtime + runtime;
							
							mHandler.post(update_data);


						//Last few images (less than a complete bunch) added to the graph file.		
						} else{
							k = new Native().map(i,stopImageNumber,imagePathText);
							res = stopImageNumber - i + 1;							
							i = stopImageNumber + 1;

							
							fps = (float)k[1]*1000/(k[0]);
							runtime = k[0];
							total_runtime = total_runtime + runtime;
							mHandler.post(update_finaldata);

							break;

						}
			
					}}
				});
				background.start();
			}
		});
	}

	//Output handling on display screen
	private Runnable update_data = new Runnable() {
		   public void run() {
			   imageCounter = imageCounter + bunchImages;
		       mainText.setText("Runtime : " + runtime);
		       secondText.setText("Number of Processed Images: " + imageCounter);
			   thirdText.setText("FPS achieved: " + fps);
			   forthText.setText("Tree file saved in: '/sdcard/Notes/mapout.bt'");
		    }
		};
	//Output handling on display screen	
	private Runnable update_finaldata = new Runnable() {
		   public void run() {
			   imageCounter = imageCounter + res;
			   average_fps = imageCounter*1000/total_runtime;
		       mainText.setText("Total Runtime : " + total_runtime);
		       secondText.setText("FINISHED - Total Number of Processed Images: " + imageCounter);
			   thirdText.setText("Average FPS achieved: " + average_fps);
			   forthText.setText("Tree file saved in: '/sdcard/Notes/mapout.bt'");
		    }
		};
	//Output handling on display screen		
	private Runnable raise_error = new Runnable() {
		   public void run() {
		       secondText.setText("Error: Stop Image must be higher than Start Image");

		    }
		};
}
