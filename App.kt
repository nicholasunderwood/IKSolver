import processing.core.PApplet
import processing.event.MouseEvent
import util.Pose2d
import util.Vector2
import kotlin.math.abs
import kotlin.math.sin
import kotlin.math.cos

import kotlin.math.acos
import kotlin.math.hypot

class App : PApplet() {

    private val arm: Arm
    private var isDragging: Boolean = false
    private var activeJoint: Joint? = null
    private var endEffector: Vector2 = Vector2();
    private var target: Vector2 = Vector2();


    init {
        val width: Float = 800f
        val height: Float = 600f

        ref = this
        arm = Arm(
            Pose2d((width / 2).toDouble(), (height - 50).toDouble(), 0.0),
            150.0, 75.0, 75.0
        )
    }

    override fun settings() {
        size(800, 600)
    }

    override fun setup() {
        getSurface().setTitle("Differential Inverse Kinematics")
        imageMode(CENTER)
        rectMode(CORNERS)
        focused = true
        this.textAlign(CENTER, CENTER)
    }

    // Continuously draws and updates the application display window
    override fun draw() {
        background(100)
        arm.tick(0.01)
        arm.draw()

        if(!isDragging) return

        fill(255f, 0f, 0f)
        val j: Joint = activeJoint ?: return
        strokeWeight(0f)
        circle((target.x).toFloat(), target.y.toFloat(), 10f)
    }

    override fun mousePressed(event: MouseEvent) {
        super.mousePressed(event)

        activeJoint = arm.isOnJointEnd(mouseX, mouseY) ?: return

        isDragging = true
        mouseDragged(event)
//        endEffector = Vector2(mouseX.toDouble(), mouseY.toDouble())
    }

    override fun mouseDragged(event: MouseEvent) {
        super.mouseDragged()
        if(!isDragging) return
        val j: Joint = activeJoint ?: return
        endEffector = Vector2(mouseX.toDouble(), mouseY.toDouble())

        target = (endEffector - j.basePose2d.pos).norm() * j.length + j.basePose2d.pos

        strokeWeight(0f); fill(255f,0f,0f)
        circle((target.x + j.basePose2d.x).toFloat(), (target.y + j.basePose2d.y).toFloat(), 5f)

        val targetPos: Double = -Math.atan((target.x - j.basePose2d.x)/(target.y - j.basePose2d.y)) - j.basePose2d.theta
        j.goToAngle(targetPos)

        println(target.dot(j.endPose2d.pos - j.basePose2d.pos))
    }

    fun calcDTheta(target: Vector2, start: Vector2, base: Vector2): Double{
        return 0.0
    }

    override fun mouseReleased() {
        super.mouseReleased()
        isDragging = false
        activeJoint = null
    }


    companion object {
        lateinit var ref: App
        const val width: Float = 800f
        const val height: Float = 600f

        fun initCanvas(args: Array<String>) {
            main("App")
        }

    }
}

fun main(args: Array<String>){
    App.initCanvas(args)
}