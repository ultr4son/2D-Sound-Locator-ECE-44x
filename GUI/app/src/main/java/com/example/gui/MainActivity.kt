package com.example.gui
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import androidx.activity.*
import androidx.camera.core.CameraSelector
import androidx.camera.core.CameraXConfig
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import com.google.common.util.concurrent.ListenableFuture
import kotlinx.android.synthetic.main.activity_main.*
import android.Manifest
import android.content.pm.PackageManager
import android.location.Location
import android.util.Log
import android.view.View.INVISIBLE
import android.view.View.VISIBLE
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.app.ActivityCompat

private const val PERMISSIONS_REQUEST_CODE = 10

class MainActivity : AppCompatActivity() {
    private lateinit var cameraProviderFuture : ListenableFuture<ProcessCameraProvider>
    private var locator: Locator? = null
    private var isLocating: Boolean = false

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
        locator = connectToLocator(this)
        initLocator()

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
            isLocating = !isLocating
            locator?.setLocationEnable(asLocationState(isLocating))
        }
        startRecordingButton.setOnClickListener {
            locator?.startRecord()
        }
        stopRecordingButton.setOnClickListener {
            locator?.stopRecord()
        }
        eraseRecordingButton.setOnClickListener {
            locator?.clearRecord()
        }

    }

    fun initLocator() {
        locator?.setLocationEnable(asLocationState(isLocating))
        locator?.setLocationMode(if(recordModeSwitch.isChecked) LocationMode.RECORDING else LocationMode.FREQUENCY)
        locator?.setFrequencyRange(frequencySelect.selectedMinValue.toInt(), frequencySelect.selectedMaxValue.toInt())
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




}