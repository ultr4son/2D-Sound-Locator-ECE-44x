package com.example.gui

import android.content.Context
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import androidx.core.content.ContextCompat.getSystemService
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialPort.*
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.nio.ByteBuffer


enum class LocationMode(val code: Byte) {
    RECORDING(0),
    FREQUENCY(1)
}
enum class LocationState(val code: Byte) {
    ON(0),
    OFF(1)
}

interface Locator {
    fun startRecord()
    fun stopRecord()
    fun clearRecord()

    fun setFrequencyRange(min: Int, max: Int)

    fun setLocationMode(mode: LocationMode)
    fun setLocationEnable(state: LocationState)
}

private const val BAUD = 115200
private const val TIMEOUT = 1000

class LocatorSerial (private val port: UsbSerialPort, private val connection: UsbDeviceConnection) : Locator {
    private enum class Command(val commandCode: Byte) {
        START_RECORD(0),
        STOP_RECORD(1),
        CLEAR_RECORD(2),
        SET_FREQUENCY_RANGE(3),
        SET_LOCATION_MODE(4),
        START_LOCATING(5),
        STOP_LOCATING(6),
    }

    init {
        port.open(connection)
        port.setParameters(BAUD, DATABITS_8, STOPBITS_1, PARITY_NONE)
    }

    override fun startRecord() {
        port.write(byteArrayOf(Command.START_RECORD.commandCode), TIMEOUT)
    }

    override fun stopRecord() {
        port.write(byteArrayOf(Command.STOP_RECORD.commandCode), TIMEOUT)
    }

    override fun clearRecord() {
        port.write(byteArrayOf(Command.CLEAR_RECORD.commandCode), TIMEOUT)
    }

    override fun setFrequencyRange(min: Int, max: Int) {
        val minBuffer = ByteBuffer.allocate(Int.SIZE_BYTES).putInt(min)
        val maxBuffer = ByteBuffer.allocate(Int.SIZE_BYTES).putInt(max)

        port.write(byteArrayOf(Command.SET_FREQUENCY_RANGE.commandCode) + minBuffer.array() + maxBuffer.array(), TIMEOUT)
    }

    override fun setLocationMode(mode: LocationMode) {
        port.write(byteArrayOf(Command.SET_LOCATION_MODE.commandCode, mode.code), TIMEOUT)
    }

    override fun setLocationEnable(state: LocationState) {
        when(state) {
            LocationState.ON -> port.write(byteArrayOf(Command.START_LOCATING.commandCode), TIMEOUT)
            LocationState.OFF -> port.write(byteArrayOf(Command.STOP_LOCATING.commandCode), TIMEOUT)
        }
    }


}

fun connectToLocator(context: Context): Locator? {
    // Find all available drivers from attached devices.
    val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager?
    val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager)
    if (availableDrivers.isEmpty()) {
        return null
    }

    // Open a connection to the first available driver.

    // Open a connection to the first available driver.
    val driver = availableDrivers[0]
    val connection = manager!!.openDevice(driver.device)
        ?:
        return null

    val port = driver.ports[0] // Most devices have just one port (port 0)
    return LocatorSerial(port, connection)
}