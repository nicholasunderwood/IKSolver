import processing.core.PApplet
import util.Matrix
import util.Pose2d
import util.Vector2
import kotlin.math.cos
import kotlin.math.sin

class Arm {
    val joints: Array<Joint>
    val dof: Int
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

    fun getJointSpace() : Matrix = Matrix(joints.size, 1){ r,_ -> joints[r].position }

    fun getEndEffector(jointSpace: Matrix): Matrix {
        var v = base.pos.clone()

        for(i in 0..jointSpace.rows){
            val dtheta = jointSpace.get(i,0)
            v += Vector2(cos(dtheta), sin(dtheta)) * joints[i].length
        }

        return Matrix(2,1){r,_ -> v.get(r) }
    }

    companion object {
        const val jointRadius: Float = 10f
        private val canvas: PApplet = App.ref
        lateinit var base : Pose2d
    }
}