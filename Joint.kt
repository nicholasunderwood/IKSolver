import processing.core.PApplet
import util.Pose2d
import kotlin.math.cos
import kotlin.math.sin

class Joint constructor(
        val parent: Joint?,
        val length: Double,
        startAngle: Double = 0.0,
        private val maxAngle: Double = Math.PI / 2,
        private val minAngle: Double = -Math.PI / 2
) {


    private val canvas: PApplet = App.ref
    private val maxSpeed = 3.0

    val basePose2d: Pose2d
        get() = parent?.endPose2d ?: Arm.base

    var endPose2d: Pose2d
        private set

    var position : Double = startAngle // [0,2pi]
    private var velocity = 0.0 // [-1,1]
    private var targetAngle = 0.0
    private var isPID = false

    init {
        endPose2d = Pose2d()
        setEndPos(startAngle)
    }

    fun setVelocity(velocity: Double) {
        this.velocity = velocity
        isPID = false
    }

    fun goToAngle(target: Double) {
        targetAngle = target.coerceIn(minAngle, maxAngle)
        isPID = true
    }

    private fun setEndPos(theta: Double) {
        val absAngle = basePose2d.theta + theta
        endPose2d = basePose2d + Pose2d(sin(absAngle), -cos(absAngle), theta) * length
    }

    fun tick(dt: Double) {
        if (isPID) {
            velocity = (targetAngle - position) * 10
            velocity = velocity.coerceIn(-1.0, 1.0)
        }

        position += dt * velocity * maxSpeed
        position = position.coerceIn(minAngle, maxAngle)

        setEndPos(position)
    }

    fun draw(jointRadius: Float) {
        canvas.circle(endPose2d.x.toFloat(), endPose2d.y.toFloat(), jointRadius)
        canvas.line(basePose2d.x.toFloat(), basePose2d.y.toFloat(), endPose2d.x.toFloat(), endPose2d.y.toFloat())


        canvas.line(
            basePose2d.x.toFloat(),
            basePose2d.y.toFloat(),
            (basePose2d.x + 30*sin(basePose2d.theta + minAngle)).toFloat(),
            (basePose2d.y - 30*cos(basePose2d.theta + minAngle)).toFloat()
        )

        canvas.line(
            basePose2d.x.toFloat(),
            basePose2d.y.toFloat(),
            (basePose2d.x + 30*sin(basePose2d.theta + maxAngle)).toFloat(),
            (basePose2d.y - 30*cos(basePose2d.theta + maxAngle)).toFloat()
        )
    }

}