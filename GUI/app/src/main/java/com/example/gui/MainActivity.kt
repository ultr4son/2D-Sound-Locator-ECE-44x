package com.example.gui
import android.Manifest
import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.camera2.CameraCharacteristics
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View.INVISIBLE
import android.view.View.VISIBLE
import android.view.ViewGroup
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.core.view.marginLeft
import androidx.lifecycle.LifecycleOwner
import com.google.common.util.concurrent.ListenableFuture
import kotlinx.android.synthetic.main.activity_main.*
import java.security.AccessController.getContext
import java.util.Collections.min
import kotlin.math.min


private const val PERMISSIONS_REQUEST_CODE = 10

class MainActivity : AppCompatActivity() {
    private lateinit var cameraProviderFuture : ListenableFuture<ProcessCameraProvider>
    private var locator: Locator? = null
    private var isLocating: Boolean = false
    private var fovX: Float = 90f
    private var fovY: Float = 90f

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
            isLocating = !isLocating
            if(!isLocating) {
                coordnatesText.visibility = INVISIBLE
                locateButton.text = "Locate"

            }
            else {
                locateButton.text = "Stop"
            }

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
    @RequiresApi(Build.VERSION_CODES.R)
    fun initCamera() {
        cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener(Runnable {
            val cameraProvider = cameraProviderFuture.get()
            bindPreview(cameraProvider)
        }, ContextCompat.getMainExecutor(this))

    }
    //Modified from: https://github.com/pchan1401-ICIL/Camera2FOV/blob/master/app/src/main/java/icil/com/camera2fov/MainActivity.java
    private fun calculateFOV(camera: Camera) {
        val characteristics =
            Camera2CameraInfo.extractCameraCharacteristics(camera.cameraInfo)
        val cOrientation = characteristics.get(CameraCharacteristics.LENS_FACING)!!
        if (cOrientation == CameraCharacteristics.LENS_FACING_BACK) {
            val maxFocus =
                characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val size =
                characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val w = size!!.width
            val h = size.height
            fovX = Math.toDegrees(
                (2 * Math.atan(w / (maxFocus!![0] * 2).toDouble()))).toFloat()
            fovY = Math.toDegrees(
                (2 * Math.atan(h / (maxFocus[0] * 2).toDouble()))).toFloat()
        }
    }


    fun bindPreview(cameraProvider : ProcessCameraProvider) {
        var preview : Preview = Preview.Builder()
            .build()

        var cameraSelector : CameraSelector = CameraSelector.Builder()
            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
            .build()
        preview.setSurfaceProvider(viewFinder.surfaceProvider)

        var camera = cameraProvider.bindToLifecycle(this as LifecycleOwner, cameraSelector, preview)
        calculateFOV(camera)

    }

    fun onRunError(e: String?) {
        Log.e(null, e.orEmpty())
        Toast.makeText(this, "Error: " + e.orEmpty(), Toast.LENGTH_SHORT).show()
    }
    fun anglesToScreenTranslation(coordnates: Pair<Int, Int>): Pair<Float, Float> {
        val boundsX = viewFinder.width
        val boundsY = viewFinder.height
        return Pair(boundsX * (coordnates.first / fovX), boundsY * (coordnates.second / fovY))
    }
    fun onCoordnates(coordnates: Pair<Int, Int>?) {
        Toast.makeText(this, "onCoordnates", Toast.LENGTH_SHORT).show()
        if(isLocating && coordnates != null) {
            if(coordnatesText.visibility == INVISIBLE) {
                coordnatesText.visibility = VISIBLE
            }


            coordnatesText.text = coordnates.first.toString() + ", " + coordnates.second.toString()
            var translation = anglesToScreenTranslation(coordnates)
            if(translation.first > viewFinder.width - coordnatesText.width || translation.second > viewFinder.height - coordnatesText.width) {
                translation = translation.copy(min(viewFinder.width.toFloat() - coordnatesText.width, translation.first), min(viewFinder.height.toFloat() - coordnatesText.height, translation.second))
                offscreenArrow.visibility = VISIBLE
            }
            else {
                offscreenArrow.visibility = INVISIBLE
            }
            Toast.makeText(this, "locating: " + translation.first.toString() + " " + translation.second.toString(), Toast.LENGTH_SHORT).show()

            val rotate = ObjectAnimator.ofFloat(offscreenArrow, "rotation", Math.toDegrees(Math.atan2(coordnates.second.toDouble() , coordnates.first.toDouble())).toFloat())
            val param = coordnatesText.layoutParams as ViewGroup.MarginLayoutParams
            param.setMargins(translation.first.toInt(), translation.second.toInt(), param.rightMargin, param.bottomMargin)
            coordnatesText.layoutParams = param
//            val animateX = ObjectAnimator.ofFloat(coordnatesText, "margin", translation.first).apply {
//                duration = 200
//                start()
//            }
//            val animateY = ObjectAnimator.ofFloat(coordnatesText, "translationY", translation.second).apply {
//                duration = 200
//                start()
//            }
            val animation = AnimatorSet()
            animation.apply {
                play(rotate)
                start()
            }
        }
    }

    fun onNewData(data: ByteArray?) {
        if(data != null && data.size > 0) {
            if(LocatorSerial.PhoneToLocatorCommand.isCommand(data[0])) {
                Toast.makeText(this, data.size.toString() + " " + LocatorSerial.PhoneToLocatorCommand.fromByte(data[0]).toString(), Toast.LENGTH_SHORT).show()
            }
            else if(LocatorSerial.LocatorToPhoneCommand.isCommand(data[0])) {
                Toast.makeText(this,
                    data.size.toString() + " " + LocatorSerial.LocatorToPhoneCommand.fromByte(data[0])
                        .toString(),
                    Toast.LENGTH_SHORT
                ).show()
            }
            else {
                Toast.makeText(this, "Invalid code " + data[0].toString(), Toast.LENGTH_SHORT).show()
            }
        }
        else {
            Toast.makeText(this, "No data", Toast.LENGTH_SHORT).show()
        }
    }

}