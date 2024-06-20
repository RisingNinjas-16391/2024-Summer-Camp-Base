package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog

class ImuInitMessage(
        @JvmField
        val deviceName: String,
        @JvmField
        val beginInit: Long,
        @JvmField
        val endInit: Long,
        @JvmField
        val timedOut: Boolean,
)

class LazyImu @JvmOverloads constructor(
        private val hardwareMap: HardwareMap,
        private val name: String,
        private val orientation: ImuOrientationOnRobot,
        private val timeoutMs: Int = 500,
) {
    private var imu: IMU? = null

    fun get(): IMU {
        if (imu == null) {
            imu = hardwareMap.get(IMU::class.java, name)
            imu!!.initialize(IMU.Parameters(orientation))

            // Try to read repeatedly until a valid quaternion is returned. Mimics the behavior of
            // resetYaw().
            val beginInit = System.nanoTime()
            val timer = ElapsedTime()
            do {
                val q = imu!!.robotOrientationAsQuaternion
                if (q.acquisitionTime != 0L) {
                    val endInit = System.nanoTime()
                    return imu!!
                }
            } while (timer.milliseconds() < timeoutMs)

            val endInit = System.nanoTime()

            RobotLog.addGlobalWarningMessage("Road Runner: IMU $name continues to return invalid data after $timeoutMs ms")

            // Better to let teams continue than to crash.
            return imu!!
        }

        return imu!!
    }
}
