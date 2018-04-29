package android.microp.bletutorial;

import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Binder;
import android.os.Environment;
import android.os.IBinder;
import android.provider.OpenableColumns;
import android.support.annotation.NonNull;
import android.support.design.widget.Snackbar;
import android.util.Log;
import android.view.View;

import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.firebase.storage.FirebaseStorage;
import com.google.firebase.storage.StorageReference;
import com.google.firebase.storage.UploadTask;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.URI;
import java.util.List;
import java.util.Random;
import java.util.UUID;


public class Service_BTLE_GATT extends Service {
    /**
     * Service for managing connection and data communication with a GATT server hosted on a
     * given Bluetooth LE device.
     */
    private final static String TAG = Service_BTLE_GATT.class.getSimpleName();

    private BluetoothManager mBluetoothManager;
    private BluetoothAdapter mBluetoothAdapter;
    private String mBluetoothDeviceAddress;
    private BluetoothGatt mBluetoothGatt;

    private int mRecordingState = STATE_CREATEFILE;
    public static final int STATE_STANDBY = 0;
    public static final int STATE_CREATEFILE = 1;
    public static final int STATE_RECORDING = 2;
    public static final int STATE_UPLOAD = 3;

    private int mConnectionState = STATE_DISCONNECTED;
    private static final int STATE_DISCONNECTED = 0;
    private static final int STATE_CONNECTING = 1;
    private static final int STATE_CONNECTED = 2;

    public final static String ACTION_GATT_CONNECTED = "android.microp.bletutorial.Service_BTLE_GATT.ACTION_GATT_CONNECTED";
    public final static String ACTION_GATT_DISCONNECTED = "android.microp.bletutorial.Service_BTLE_GATT.ACTION_GATT_DISCONNECTED";
    public final static String ACTION_GATT_SERVICES_DISCOVERED = "android.microp.bletutorial.Service_BTLE_GATT.ACTION_GATT_SERVICES_DISCOVERED";
    public final static String ACTION_DATA_AVAILABLE = "android.microp.bletutorial.Service_BTLE_GATT.ACTION_DATA_AVAILABLE";
    public final static String EXTRA_UUID = "android.microp.bletutorial.Service_BTLE_GATT.EXTRA_UUID";
    public final static String EXTRA_DATA = "android.microp.bletutorial.Service_BTLE_GATT.EXTRA_DATA";

    private static File file;
    private static int fileNum;
    private StorageReference mStorageRef;
    private static View coordinatorLayout;
    private Activity_BTLE_Services activity_btle_services;
    private Intent myIntent;

    private byte[] fileBuffer = new byte[0];

    // Implements callback methods for GATT events that the app cares about.  For example,
    // connection change and services discovered.
    private final BluetoothGattCallback mGattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            String intentAction;
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                intentAction = ACTION_GATT_CONNECTED;

                mConnectionState = STATE_CONNECTED;

                broadcastUpdate(intentAction);

                Log.i(TAG, "Connected to GATT server.");
                // Attempts to discover services after successful connection.
                Log.i(TAG, "Attempting to start service discovery:" + mBluetoothGatt.discoverServices());
            }
            else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                intentAction = ACTION_GATT_DISCONNECTED;

                mConnectionState = STATE_DISCONNECTED;

                Log.i(TAG, "Disconnected from GATT server.");

                broadcastUpdate(intentAction);
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            Log.i(TAG, "TAG3 : " + "onServicesDiscovered" +"\n");
            if (status == BluetoothGatt.GATT_SUCCESS) {
                broadcastUpdate(ACTION_GATT_SERVICES_DISCOVERED);
            }
            else {
                Log.w(TAG, "onServicesDiscovered received: " + status);
            }
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
            Log.i(TAG, "TAG2" + characteristic.getStringValue(0)+"\n");
            if (status == BluetoothGatt.GATT_SUCCESS) {
                broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);
            }
        }



        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            UUID audioDataUUID = UUID.fromString("03366e80-cf3a-11e1-9ab4-0002a5d5c51b");
            UUID recordingUUID = UUID.fromString("b071ac21-2477-11e2-82d0-0002a5d5c51b");
            Log.i(TAG, "TAG1" + characteristic.getStringValue(0)+"\n");


            if(audioDataUUID.equals(characteristic.getUuid())){

                if(fileBuffer.length == 0) {
                    Log.i(TAG, "TAG7 - starting upload timer");
                    startUploadTimer();
                }

                // add to file fileBuffer
                fileBuffer = concatenateByteArrays(fileBuffer, characteristic.getValue());

            }
            //createFile(characteristic.getValue());
            broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);
        }

        byte[] concatenateByteArrays(byte[] a, byte[] b) {
            byte[] result = new byte[a.length + b.length];
            System.arraycopy(a, 0, result, 0, a.length);
            System.arraycopy(b, 0, result, a.length, b.length);
            return result;
        }

        public void createFile(byte[] array){
            if(mRecordingState != STATE_UPLOAD) {
                createFile(mRecordingState, array);
            }else{
                mRecordingState = STATE_STANDBY;
                createNewWaveFile();
                uploadFile();
            }

        }
        //Creates file in a local storage
        public void createFile(int state, byte[] array){
            if(state == Service_BTLE_GATT.STATE_STANDBY){

            }else if(state == Service_BTLE_GATT.STATE_UPLOAD){
                uploadFile();
            }else {
                if(state == Service_BTLE_GATT.STATE_CREATEFILE){
                    fileNum =  new Random().nextInt(500);
                    file = new File(Environment.getExternalStoragePublicDirectory(
                            Environment.DIRECTORY_DOCUMENTS), "myfile");

                        Log.i(TAG, "TAG6 - creating file yoo");

                        StorageReference ourFileRef = mStorageRef.child("ECSE426/" + fileNum + ".wav");
                        UploadTask uploadTask = ourFileRef.putBytes(array);
                        uploadTask.addOnFailureListener(new OnFailureListener() {
                            @Override
                            public void onFailure(@NonNull Exception exception) {
                                // Handle unsuccessful uploads
                                Log.i(TAG, "TAG6 - I failed in uploading the file :(");
                                fileBuffer = new byte[0];
                            }
                        }).addOnSuccessListener(new OnSuccessListener<UploadTask.TaskSnapshot>(){
                            @Override
                            public void onSuccess(UploadTask.TaskSnapshot taskSnapshot) {
                                // taskSnapshot.getMetadata() contains file metadata such as size, content-type, and download URL.
                                Uri downloadUrl = taskSnapshot.getDownloadUrl();
                                Log.i(TAG, "TAG6 - Successfuly uploaded the file");
                                fileBuffer = new byte[0];
                            }
                        });

                        // if(!file.createNewFile()){
                        //     Log.i(TAG, "TAG7 - delete file");
                        //     file.delete();
                        //     file.createNewFile();
                        // } else {
                        //   Log.i(TAG, "TAG8 - uploading file");
                        //   uploadFile();
                        // }

                }else{
                    if(file == null) {
                        file = new File(Environment.getExternalStoragePublicDirectory(
                                Environment.DIRECTORY_DOCUMENTS), "myfile");
                    }

                }

                try {
                    FileOutputStream outStream = new FileOutputStream(file, true);
                    DataOutputStream outStreamWriter = new DataOutputStream(outStream);

                    outStreamWriter.write(array,0,array.length);
                    outStreamWriter.flush();
                } catch (Exception e) {

                }
            }
        }

        public void startUploadTimer(){
          new java.util.Timer().schedule(
          new java.util.TimerTask() {
            @Override
            public void run() {
                createFile(fileBuffer);
            }
          },
          1000
          );
        }

        //Creates the Wave file from 8 bits PCM file
        public void createNewWaveFile(){
            Snackbar.make(coordinatorLayout, "Create Wave File", Snackbar.LENGTH_LONG)
                    .setAction("Action", null).show();
            byte[] data = getByte(file);
            fileNum =  new Random().nextInt(500);
            file = new File(Environment.getExternalStoragePublicDirectory(
                    Environment.DIRECTORY_DOCUMENTS), "myfile" + fileNum + ".wav");
            try {
                Log.i(TAG, "TAG - creating wav file");
                if(!file.createNewFile()){
                    file.delete();
                    file.createNewFile();
                }

            } catch (IOException e) {
                e.printStackTrace();
            }

            try {
                FileOutputStream outStream = new FileOutputStream(file, true);
                DataOutputStream outStreamWriter = new DataOutputStream(outStream);

                PCMtoFile(outStream,data,8000,1,8);
            } catch (Exception e) {

            }
        }

        //Retrieves file from your android device
        private byte[] getByte(File f){
            byte fileContent[] = new byte[(int)f.length()];
            FileInputStream fin = null;
            try {
                // create FileInputStream object
                fin = new FileInputStream(f);

                // Reads up to certain bytes of data from this input stream into an array of bytes.
                fin.read(fileContent);
                //create string from byte array
                String s = new String(fileContent);
                System.out.println("File content: " + s);
            }
            catch (FileNotFoundException e) {
                System.out.println("File not found" + e);
            }
            catch (IOException ioe) {
                System.out.println("Exception while reading file " + ioe);
            }
            finally {
                // close the streams using close method
                try {
                    if (fin != null) {
                        fin.close();
                    }
                }
                catch (IOException ioe) {
                    System.out.println("Error while closing stream: " + ioe);
                }
            }

            return fileContent;
        }


        //Gets file name by removing the directory of the file
        public String getFileName(Uri uri) {
            String result = null;
            if (uri.getScheme().equals("content")) {
                Cursor cursor = getContentResolver().query(uri, null, null, null, null);
                try {
                    if (cursor != null && cursor.moveToFirst()) {
                        result = cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME));
                    }
                } finally {
                    cursor.close();
                }
            }
            if (result == null) {
                result = uri.getPath();
                int cut = result.lastIndexOf('/');
                if (cut != -1) {
                    result = result.substring(cut + 1);
                }
            }
            return result;
        }

        //Add the Wave file header to the PCM file
        public void PCMtoFile(OutputStream os, byte[] data, int srate, int channel, int format) throws IOException {
            byte[] header = new byte[44];

            long totalDataLen = data.length + 36;
            long bitrate = srate * channel * format;

            header[0] = 'R';
            header[1] = 'I';
            header[2] = 'F';
            header[3] = 'F';
            header[4] = (byte) (totalDataLen & 0xff);
            header[5] = (byte) ((totalDataLen >> 8) & 0xff);
            header[6] = (byte) ((totalDataLen >> 16) & 0xff);
            header[7] = (byte) ((totalDataLen >> 24) & 0xff);
            header[8] = 'W';
            header[9] = 'A';
            header[10] = 'V';
            header[11] = 'E';
            header[12] = 'f';
            header[13] = 'm';
            header[14] = 't';
            header[15] = ' ';
            header[16] = 16;
            header[17] = 0;
            header[18] = 0;
            header[19] = 0;
            header[20] = 1;
            header[21] = 0;
            header[22] = (byte) channel;
            header[23] = 0;
            header[24] = (byte) (srate & 0xff);
            header[25] = (byte) ((srate >> 8) & 0xff);
            header[26] = (byte) ((srate >> 16) & 0xff);
            header[27] = (byte) ((srate >> 24) & 0xff);
            header[28] = (byte) ((bitrate / 8) & 0xff);
            header[29] = (byte) (((bitrate / 8) >> 8) & 0xff);
            header[30] = (byte) (((bitrate / 8) >> 16) & 0xff);
            header[31] = (byte) (((bitrate / 8) >> 24) & 0xff);
            header[32] = (byte) ((channel * format) / 8);
            header[33] = 0;
            header[34] = 8;
            header[35] = 0;
            header[36] = 'd';
            header[37] = 'a';
            header[38] = 't';
            header[39] = 'a';
            header[40] = (byte) (data.length  & 0xff);
            header[41] = (byte) ((data.length >> 8) & 0xff);
            header[42] = (byte) ((data.length >> 16) & 0xff);
            header[43] = (byte) ((data.length >> 24) & 0xff);
            os.write(header, 0, 44);
            os.write(data);
            os.close();
        }

        //Upload file to Firebase
        public void uploadFile(){
            Snackbar.make(coordinatorLayout, "Uploading ...", Snackbar.LENGTH_LONG)
                    .setAction("Action", null).show();
            URI oldUri = file.toURI();
            Uri filePath  = new Uri.Builder().scheme(oldUri.getScheme())
                    .encodedAuthority(oldUri.getRawAuthority())
                    .encodedPath(oldUri.getRawPath())
                    .query(oldUri.getRawQuery())
                    .fragment(oldUri.getRawFragment())
                    .build();
            String f = getFileName(filePath);
            StorageReference riversRef = mStorageRef.child("ECSE426/" + f);

            riversRef.putFile(filePath)
                    .addOnSuccessListener(new OnSuccessListener<UploadTask.TaskSnapshot>() {
                        @Override
                        public void onSuccess(UploadTask.TaskSnapshot taskSnapshot) {
                            // Get a URL to the uploaded content
                            Snackbar.make(coordinatorLayout, "File Uploaded", Snackbar.LENGTH_LONG)
                                    .setAction("Action", null).show();
                            Uri downloadUrl = taskSnapshot.getDownloadUrl();
                        }
                    })
                    .addOnFailureListener(new OnFailureListener() {
                        @Override
                        public void onFailure(@NonNull Exception exception) {
                            // Handle unsuccessful uploads
                            Snackbar.make(coordinatorLayout, "Upload Failed", Snackbar.LENGTH_LONG)
                                    .setAction("Action", null).show();
                            // ...
                        }
                    });
        }


    };

    private void broadcastUpdate(final String action) {
        final Intent intent = new Intent(action);
        sendBroadcast(intent);
    }

    private void broadcastUpdate(final String action, final BluetoothGattCharacteristic characteristic) {

        final Intent intent = new Intent(action);

        intent.putExtra(EXTRA_UUID, characteristic.getUuid().toString());

        // For all other profiles, writes the data formatted in HEX.
        final byte[] data = characteristic.getValue();

        if (data != null && data.length > 0) {

            intent.putExtra(EXTRA_DATA, new String(data) + "\n" + Utils.hexToString(data));
        }
        else {
            intent.putExtra(EXTRA_DATA, "0");
        }

        sendBroadcast(intent);
    }

    public class BTLeServiceBinder extends Binder {

        Service_BTLE_GATT getService() {
            return Service_BTLE_GATT.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    @Override
    public void onCreate() {
        mStorageRef = FirebaseStorage.getInstance().getReference();
        activity_btle_services = Activity_BTLE_Services.instance;
        coordinatorLayout = activity_btle_services.findViewById(android.R.id.content);

    }

    @Override
    public boolean onUnbind(Intent intent) {
        // After using a given device, you should make sure that BluetoothGatt.close() is called
        // such that resources are cleaned up properly.  In this particular example, close() is
        // invoked when the UI is disconnected from the Service.
        close();
        return super.onUnbind(intent);
    }

    private final IBinder mBinder = new BTLeServiceBinder();

    /**
     * Initializes a reference to the local Bluetooth adapter.
     *
     * @return Return true if the initialization is successful.
     */
    public boolean initialize() {
        // For API level 18 and above, get a reference to BluetoothAdapter through
        // BluetoothManager.
        if (mBluetoothManager == null) {
            mBluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
            if (mBluetoothManager == null) {
                Log.e(TAG, "Unable to initialize BluetoothManager.");
                return false;
            }
        }

        mBluetoothAdapter = mBluetoothManager.getAdapter();
        if (mBluetoothAdapter == null) {
            Log.e(TAG, "Unable to obtain a BluetoothAdapter.");
            return false;
        }

        return true;
    }



    /**
     * Connects to the GATT server hosted on the Bluetooth LE device.
     *
     * @param address The device address of the destination device.
     *
     * @return Return true if the connection is initiated successfully. The connection result
     *         is reported asynchronously through the
     *         {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     *         callback.
     */
    public boolean connect(final String address) {

        if (mBluetoothAdapter == null || address == null) {
            Log.w(TAG, "BluetoothAdapter not initialized or unspecified address.");
            return false;
        }

        // Previously connected device.  Try to reconnect.
        if (mBluetoothDeviceAddress != null && address.equals(mBluetoothDeviceAddress) && mBluetoothGatt != null) {
            Log.d(TAG, "Trying to use an existing mBluetoothGatt for connection.");

            if (mBluetoothGatt.connect()) {
                mConnectionState = STATE_CONNECTING;
                return true;
            }
            else {
                return false;
            }
        }

        final BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);

        if (device == null) {
            Log.w(TAG, "Device not found.  Unable to connect.");
            return false;
        }

        // We want to directly connect to the device, so we are setting the autoConnect
        // parameter to false.
        mBluetoothGatt = device.connectGatt(this, false, mGattCallback);
        Log.d(TAG, "Trying to create a new connection.");
        mBluetoothDeviceAddress = address;
        mConnectionState = STATE_CONNECTING;

        return true;
    }

    /**
     * Disconnects an existing connection or cancel a pending connection. The disconnection result
     * is reported asynchronously through the
     * {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     * callback.
     */
    public void disconnect() {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }

        mBluetoothGatt.disconnect();
    }

    /**
     * After using a given BLE device, the app must call this method to ensure resources are
     * released properly.
     */
    public void close() {

        if (mBluetoothGatt == null) {
            return;
        }

        mBluetoothGatt.close();
        mBluetoothGatt = null;
    }

    /**
     * Request a read on a given {@code BluetoothGattCharacteristic}. The read result is reported
     * asynchronously through the {@code BluetoothGattCallback#onCharacteristicRead(android.bluetooth.BluetoothGatt, android.bluetooth.BluetoothGattCharacteristic, int)}
     * callback.
     *
     * @param characteristic The characteristic to read from.
     */
    public void readCharacteristic(BluetoothGattCharacteristic characteristic) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }

        mBluetoothGatt.readCharacteristic(characteristic);
    }

    /**
     * Request a write on a given {@code BluetoothGattCharacteristic}. The write result is reported
     * asynchronously through the {@code BluetoothGattCallback#onCharacteristicWrite(android.bluetooth.BluetoothGatt, android.bluetooth.BluetoothGattCharacteristic, int)}
     * callback.
     *
     * @param characteristic The characteristic to read from.
     */
    public void writeCharacteristic(BluetoothGattCharacteristic characteristic) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }

        mBluetoothGatt.writeCharacteristic(characteristic);
    }

    /**
     * Enables or disables notification on a give characteristic.
     *
     * @param characteristic Characteristic to act on.
     * @param enabled If true, enable notification.  False otherwise.
     */
    public void setCharacteristicNotification(BluetoothGattCharacteristic characteristic, boolean enabled) {

        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }

        mBluetoothGatt.setCharacteristicNotification(characteristic, enabled);

        BluetoothGattDescriptor descriptor = characteristic.getDescriptor(
                UUID.fromString(getString(R.string.CLIENT_CHARACTERISTIC_CONFIG)));

        if (enabled) {
            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
        }
        else {
            descriptor.setValue(BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE);
        }


        mBluetoothGatt.writeDescriptor(descriptor);
    }

    /**
     * Retrieves a list of supported GATT services on the connected device. This should be
     * invoked only after {@code BluetoothGatt#discoverServices()} completes successfully.
     *
     * @return A {@code List} of supported services.
     */
    public List<BluetoothGattService> getSupportedGattServices() {

        if (mBluetoothGatt == null) {
            return null;
        }

        return mBluetoothGatt.getServices();
    }
}
