import processing.core.PApplet
import processing.event.MouseEvent
import util.Pose2d
import util.Vector2
import java.lang.RuntimeException
import kotlin.math.abs
import kotlin.math.sin
import kotlin.math.cos

import kotlin.math.acos
import kotlin.math.hypot

class App : PApplet() {

    private val arm: Arm
    private val ikSolver: IKSolver
    private var isDragging: Boolean = false
    private var activeJoint: Joint? = null
    private var endEffector: Vector2 = Vector2();
    private var target: Vector2 = Vector2();

    private val endEffectorD: Double = 3.0;
    private val dX: Vector2 = Vector2(endEffectorD, 0.0)
    private val dY: Vector2 = Vector2(0.0, -endEffectorD)
    private var dTarget: Vector2 = Vector2()


    init {
        val width: Float = 900f
        val height: Float = 900f

        ref = this
        arm = Arm(
            Pose2d((width / 2).toDouble(), (height / 2).toDouble(), 0.0),
            150.0, 75.0, 75.0
        )
        ikSolver = IKSolver(arm)
    }

    override fun settings() {
        size(900, 900)
    }

    override fun setup() {
        getSurface().setTitle("Differential Inverse Kinematics")
        imageMode(CENTER)
        rectMode(CORNERS)
        focused = true
        this.textAlign(CENTER, CENTER)

        endEffector = arm.joints.last().endPose2d.pos
    }

    // Continuously draws and updates the application display window
    override fun draw() {
        background(100)
        arm.tick(0.01)
        arm.draw()

        noFill()
        circle(Arm.base.x.toFloat(), Arm.base.y.toFloat(), 2 * arm.joints.sumByDouble { it.length }.toFloat())

        strokeWeight(0f)
        fill(255f, 0f, 0f)
        endEffector += dTarget
        endEffector.draw(10f)

        target = (endEffector - Arm.base.pos).unit()
        println(target)

        if(arm.getEndEffector().dist(target) > 2.0){
            var dx: DoubleArray = try { ikSolver.getIK(target, 5000) } catch (e: IllegalArgumentException) { print(e.message); return }

            for(i in (0..0)){
                break;
                dx = ikSolver.refineSolution(target, dx)
            }
            print(dx)
            arm.joints.forEachIndexed { i,j -> j.setVelocity(dx[i])}
//            arm.joints.forEachIndexed { i,j -> j.setAngle(j.position + dx[i])}
//            arm.joints.forEachIndexed  { i,j -> j.setAngle(j.position + dx[i])}

        } else {
            arm.joints.forEach { it.setVelocity(0.0)}
        }


//        var i = 0;
//        val ty = arm.joints.sumByDouble { j -> i++; Math.cos(arm.joints.copyOfRange(0, i).sumByDouble{ it.position }) * j.length  }
//        i = 0;
//        val tx = arm.joints.sumByDouble { j -> i++; Math.sin(arm.joints.copyOfRange(0, i).sumByDouble{ it.position }) * j.length  }
//
//        Arm.base.pos.plus(Vector2(tx,-ty)).draw(20f);

//        if(!isDragging) return

//        val j: Joint = activeJoint ?: return
//        circle((target.x).toFloat(), target.y.toFloat(), 10f)

    }

    override fun mousePressed(event: MouseEvent) {
        super.mousePressed(event)

        if(Vector2(this.mouseX, this.mouseY).dist(endEffector) < 10) return

        isDragging = true
        mouseDragged(event)
//        endEffector = Vector2(mouseX.toDouble(), mouseY.toDouble())
    }

    override fun mouseDragged(event: MouseEvent) {
        super.mouseDragged()
        endEffector = Vector2(mouseX.toDouble(), mouseY.toDouble())

        target = endEffector - Arm.base.pos

        strokeWeight(0f); fill(255f,0f,0f)
    }

    override fun keyPressed() {
        super.keyPressed()
        when (keyCode) {
            UP -> {
                dTarget += dY
            }
            DOWN -> {
                dTarget -= dY
            }
            RIGHT -> {
                dTarget += dX
            }
            LEFT -> {
                dTarget -= dX
            }
        }
    }

    override fun keyReleased() {
        super.keyReleased()
        when (keyCode) {
            UP -> {
                dTarget -= dY
            }
            DOWN -> {
                dTarget += dY
            }
            RIGHT -> {
                dTarget -= dX
            }
            LEFT -> {
                dTarget += dX
            }
        }
    }

    fun calcDTheta(target: Vector2, start: Vector2, base: Vector2): Double{
        return 0.0
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