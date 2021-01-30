package com.example.gui

import android.app.Activity
import android.content.Context
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialPort.*
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.lang.Exception
import java.nio.ByteBuffer
import java.util.concurrent.Executors


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

    fun setFrequencyRange(min: Short, max: Short)

    fun setLocationMode(mode: LocationMode)
    fun setLocationEnable(state: LocationState)

}

private const val BAUD = 115200
private const val TIMEOUT = 1000

class LocatorSerial (val context: Activity,val port: UsbSerialPort, connection: UsbDeviceConnection, val onCoordnates: (Pair<Int, Int>?) -> Unit, val onError: (String) -> Unit, val onRecordingStopped: ()->Unit, val onOther: (ByteArray?) -> Unit = {}) : Locator, SerialInputOutputManager.Listener {
    //The next enums describe the possible commands that will be sent between the phone and locator.
    //In general, the transmission format follows:

    //Phone to locator:
    //1. COMMAND_CODE (1 byte)
    //2. COMMAND_CODE | ARG 1 (1 byte + 1 byte)
    //3. COMMAND_CODE | ARG 1 | ARG 2 (1 byte + 2 bytes + 2 bytes)

    //Locator to phone:
    //1. COMMAND_CODE | ANGLE_X | ANGLE_Y (1 byte + 2 bytes + 2 bytes)
    //2. COMMAND_CODE | Null-terminated string (1 byte + n bytes) TODO: not essential right now, implement this later
    //3. COMMAND_CODE (1 byte) TODO: Not essential right now, implement this later
    //Command codes will be separated between the phone and locator for now because this allows for very simple verification that the command code has been sent to the locator (locator just echoes back the code it recived)

    //Description of possible commands that the phone will send to the locator and command format <ARGUMENT1(bytes) | ARGUMENT2(bytes) | ...>
    enum class PhoneToLocatorCommand(val commandCode: Byte) {
        //Begin recording audio: COMMAND_CODE(1)
        START_RECORD(0),
        //Stop recording audio and store audio clip: COMMAND_CODE(1)
        STOP_RECORD(1),
        //Clear audio clip: COMMAND_CODE(1)
        CLEAR_RECORD(2),
        //Set frequency detection range between MIN and MAX: COMMAND_CODE(1) | MIN(2) | MAX(2)
        SET_FREQUENCY_RANGE(3),
        //Set location mode between recording and frequency mode (see location mode enum): COMMAND_CODE(1) | RECORDING_OR_FREQUENCY(1)
        SET_LOCATION_MODE(4),
        //Begin locating: COMMAND_CODE(1)
        START_LOCATING(5),
        //Stop locating: COMMAND_CODE(1)
        STOP_LOCATING(6);

        companion object {
            private val map = values().associateBy(PhoneToLocatorCommand::commandCode)
            fun fromByte(code: Byte) = map[code]
            fun isCommand(code: Byte) = code in map
        }
    }
    enum class LocatorToPhoneCommand(val commandCode: Byte) {
        //Set angle display to new angle: COMMAND_CODE(1) | X(2) | Y(2)
        SET_COORDNATES(7);
        companion object {
            private val map = values().associateBy(LocatorToPhoneCommand::commandCode)
            fun fromByte(code: Byte) = map[code]
            fun isCommand(code: Byte) = code in map
        }

    }

    init {
        port.open(connection)
        port.setParameters(BAUD, DATABITS_8, STOPBITS_1, PARITY_NONE)
        port.setDTR(true); // for arduino, ...
        port.setRTS(true);
        val ioManager = SerialInputOutputManager(port, this)
        Executors.newSingleThreadExecutor().submit(ioManager)

    }

    override fun onNewData(data: ByteArray?) {
        if(data != null && data.size > 0) {
            if(data[0].equals(LocatorToPhoneCommand.SET_COORDNATES.commandCode)) {
                context.runOnUiThread {onCoordnates(bytesToAngles(data.sliceArray(IntRange(1, data.size - 1))))}

            }
            else {
                context.runOnUiThread {onOther(data)}
            }

        }
    }

    override fun onRunError(e: Exception?) {
        context.runOnUiThread {
            onError(e?.message.orEmpty())
        }
    }
    override fun startRecord() {
        port.write(byteArrayOf(PhoneToLocatorCommand.START_RECORD.commandCode), TIMEOUT)
    }

    override fun stopRecord() {
        port.write(byteArrayOf(PhoneToLocatorCommand.STOP_RECORD.commandCode), TIMEOUT)
    }

    override fun clearRecord() {
        port.write(byteArrayOf(PhoneToLocatorCommand.CLEAR_RECORD.commandCode), TIMEOUT)
    }

    override fun setFrequencyRange(min: Short, max: Short) {
        val minBuffer = ByteBuffer.allocate(Short.SIZE_BYTES).putShort(min)
        val maxBuffer = ByteBuffer.allocate(Short.SIZE_BYTES).putShort(max)

        port.write(byteArrayOf(PhoneToLocatorCommand.SET_FREQUENCY_RANGE.commandCode) + minBuffer.array() + maxBuffer.array(), TIMEOUT)
    }

    override fun setLocationMode(mode: LocationMode) {
        port.write(byteArrayOf(PhoneToLocatorCommand.SET_LOCATION_MODE.commandCode, mode.code), TIMEOUT)
    }

    override fun setLocationEnable(state: LocationState) {
        when(state) {
            LocationState.ON -> port.write(byteArrayOf(PhoneToLocatorCommand.START_LOCATING.commandCode), TIMEOUT)
            LocationState.OFF -> port.write(byteArrayOf(PhoneToLocatorCommand.STOP_LOCATING.commandCode), TIMEOUT)
        }
    }




}
fun bytesToAngles(bytes:ByteArray): Pair<Int, Int>? {
    if(bytes.size != Short.SIZE_BYTES * 2) {
        return null
    }

    val x = ByteBuffer.wrap(bytes.sliceArray(IntRange(0, Short.SIZE_BYTES - 1))).short.toInt()
    val y = ByteBuffer.wrap(bytes.sliceArray(IntRange(Short.SIZE_BYTES, Short.SIZE_BYTES * 2 - 1))).short.toInt()

    return Pair<Int,Int>(x, y)
}

/**
 * Create a connection between the phone and locator device
 * @param context: main activity context
 * @param onCoordnates: Called when the locator sends a pair of coordnates
 * @param onError: Called when the locator sends an error code
 * @param onOther: Called when the locator sends something that is not coordnates or an error code
 * @return A Locator interface object
 */
fun connectToLocator(context: Context, onCoordnates: (Pair<Int, Int>?) -> Unit, onError: (String?) -> Unit, onOther: (ByteArray?) -> Unit): Locator? {
    // Find all available drivers from attached devices.
    val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager?
    val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager)
    if (availableDrivers.isEmpty()) {
        return null
    }

    // Open a connection to the first available driver.
    val driver = availableDrivers[0]
    val connection = manager!!.openDevice(driver.device)
        ?:
        return null

    val port = driver.ports[0] // Most devices have just one port (port 0)
    return LocatorSerial(context as Activity, port, connection, onCoordnates, onError, {}, onOther)
}