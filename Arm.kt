import processing.core.PApplet
import util.Pose2d
import util.Vector2
import kotlin.math.cos
import kotlin.math.sin
import Jama.Matrix

class Arm {
    val joints: Array<Joint>
    val axes: Int
        get() = joints.size

    private val canvas: PApplet = App.ref
    val maxLength: Double

    constructor(vararg mJoints: Joint) {
        joints = mJoints as Array<Joint>
        var length = 0.0
        for(j: Joint in mJoints){
            length += j.length
        }
        maxLength = length
    }

    constructor(base: Pose2d, vararg lengths: Double) {
        Arm.base = base
        var length = 0.0
        var lastJoint: Joint? = null;

        joints = Array<Joint>(lengths.size) {
            lastJoint = Joint(lastJoint, lengths[it]);
            length += lastJoint!!.length
            lastJoint!!
        }

        maxLength = length
    }

    fun tick(dt: Double) {
        for (j in joints) {
            j.tick(dt)
        }
    }

    fun draw() {
        canvas.fill(0)
        canvas.strokeWeight(5f)

        canvas.circle(base.x.toFloat(), base.y.toFloat(), 15f)
        for (joint in joints) joint.draw(jointRadius)
    }

    fun isOnJointEnd(x: Int, y: Int): Joint? {
        for(joint in joints){
            if(Math.hypot(x -joint.endPose2d.x, y - joint.endPose2d.y) < jointRadius)
                return joint
        }
        return null
    }

    fun getJointPositions() : Array<Double> = Array(joints.size) { joints[it].position }

    fun getJointSpace() : Matrix {
        val m = Array(1){ Array(joints.size){ joints[it].position }}
        return Matrix(2,1,0.0)
    }

    fun getEndEffector(jointSpace: DoubleArray): Vector2 {
        var v = Vector2()
        var m: Array<out Array<Double>> = Array(1){Array(2){0.0}}

        for(i in 0..jointSpace.size){
            v += Vector2(cos(jointSpace[i]), sin(jointSpace[i])) * joints[i].length
        }

        return v

    }

    companion object {
        const val jointRadius: Float = 10f
        private val canvas: PApplet = App.ref
        lateinit var base : Pose2d
    }
}