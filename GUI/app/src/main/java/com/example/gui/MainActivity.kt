package com.example.gui
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import androidx.camera.core.CameraSelector
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import com.google.common.util.concurrent.ListenableFuture
import kotlinx.android.synthetic.main.activity_main.*
import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Camera
import android.hardware.camera2.CameraManager
import android.hardware.camera2.params.StreamConfigurationMap
import android.util.Log
import android.view.View.INVISIBLE
import android.view.View.VISIBLE
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.Camera2Interop
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.lang.Exception

private const val PERMISSIONS_REQUEST_CODE = 10

class MainActivity : AppCompatActivity() {
    private lateinit var cameraProviderFuture : ListenableFuture<ProcessCameraProvider>
    private var locator: Locator? = null
    private var isLocating: Boolean = false
    private var fovX: Int = 90
    private var fovY: Int = 90

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val requestPermissionLauncher =
            registerForActivityResult(
                ActivityResultContracts.RequestPermission()
            ) { isGranted: Boolean ->
                if (isGranted) {
                    initCamera()
                }
            }

        when(ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.CAMERA
        )) {
            PackageManager.PERMISSION_GRANTED -> {

                initCamera()
            }
            else -> {
                // You can directly ask for the permission.
                // The registered ActivityResultCallback gets the result of this request.
                requestPermissionLauncher.launch(
                    Manifest.permission.CAMERA)
            }
        }
        locator = connectToLocator(this,::onCoordnates, ::onRunError, ::onNewData)
        if(locator != null) {
            initLocator()
            Toast.makeText(this, "Locator connected", Toast.LENGTH_SHORT).show()
        }

        recordModeSwitch.setOnClickListener {
            if(recordModeSwitch.isChecked) {
                startRecordingButton.visibility = VISIBLE;
                stopRecordingButton.visibility = VISIBLE;
                eraseRecordingButton.visibility = VISIBLE;

                frequencySelect.visibility = INVISIBLE

                locator?.setLocationMode(LocationMode.RECORDING)
            }
            else  {
                startRecordingButton.visibility = INVISIBLE;
                stopRecordingButton.visibility = INVISIBLE;
                eraseRecordingButton.visibility = INVISIBLE;

                frequencySelect.visibility = VISIBLE;

                locator?.setLocationMode(LocationMode.FREQUENCY)
            }
        }
        locateButton.setOnClickListener {
            Toast.makeText(this, "wow", Toast.LENGTH_SHORT).show()
            isLocating = !isLocating
            if(!isLocating) {
                coordnatesText.visibility = INVISIBLE
            }
            locator?.setLocationEnable(asLocationState(isLocating))

        }
        startRecordingButton.setOnClickListener {
            Toast.makeText(this, "recording", Toast.LENGTH_SHORT).show()
            locator?.startRecord()
        }
        stopRecordingButton.setOnClickListener {
            locator?.stopRecord()
        }
        eraseRecordingButton.setOnClickListener {
            locator?.clearRecord()
        }
        frequencySelect.setOnRangeSeekBarChangeListener { bar, number, number2 ->
            locator?.setFrequencyRange(number.toShort(), number2.toShort())
        }

    }
    override fun onNewIntent(intent: Intent) {
        Toast.makeText(this, "Intent", Toast.LENGTH_SHORT).show()
        if(intent.action.equals("android.hardware.usb.action.USB_DEVICE_ATTACHED")) {
            Toast.makeText(this, "Usb device", Toast.LENGTH_SHORT).show()

            locator = connectToLocator(this, ::onCoordnates , ::onRunError, ::onNewData)
            if(locator != null) {
                initLocator()
                Toast.makeText(this, "Locator connected", Toast.LENGTH_SHORT).show()
            }
        }
        super.onNewIntent(intent)

    }

    fun initLocator() {
        locator?.setLocationEnable(asLocationState(isLocating))
        locator?.setLocationMode(if(recordModeSwitch.isChecked) LocationMode.RECORDING else LocationMode.FREQUENCY)
        locator?.setFrequencyRange(frequencySelect.selectedMinValue.toShort(), frequencySelect.selectedMaxValue.toShort())
        locator?.stopRecord()
        locator?.clearRecord()
    }
    fun asLocationState(locating: Boolean): LocationState {
        return if(locating) LocationState.ON else LocationState.OFF
    }
    fun initCamera() {
        cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener(Runnable {
            val cameraProvider = cameraProviderFuture.get()
            bindPreview(cameraProvider)
        }, ContextCompat.getMainExecutor(this))

    }


    fun bindPreview(cameraProvider : ProcessCameraProvider) {
        var preview : Preview = Preview.Builder()
            .build()

        var cameraSelector : CameraSelector = CameraSelector.Builder()
            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
            .build()
        preview.setSurfaceProvider(viewFinder.surfaceProvider)

        var camera = cameraProvider.bindToLifecycle(this as LifecycleOwner, cameraSelector, preview)



    }

    fun onRunError(e: String?) {
        Log.e(null, e.orEmpty())
    }
    fun onCoordnates(coordnates: Pair<Int, Int>) {
        if(isLocating) {
            if(coordnatesText.visibility == INVISIBLE) {
                coordnatesText.visibility = VISIBLE
            }
            coordnatesText.text = coordnates.first.toString() + ", " + coordnates.second.toString()
            coordnatesText.x = coordnates.first.toFloat()
            coordnatesText.y = coordnates.second.toFloat()

        }
}
    fun onNewData(data: ByteArray) {
        if(data.size > 0) {
            if(LocatorSerial.PhoneToLocatorCommand.isCommand(data[0])) {
                Toast.makeText(this, data.size.toString() + " " + LocatorSerial.PhoneToLocatorCommand.fromByte(data[0]).toString(), Toast.LENGTH_SHORT).show()
            }
        }
    }

}