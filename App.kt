import processing.core.PApplet
import processing.event.MouseEvent
import util.Pose2d
import util.Vector2
import kotlin.math.sign

class App : PApplet() {

    private val arm: Arm
    private var useIK: Boolean = false
    private val ikSolver: IKSolver
    private var isDragging: Boolean = false
    private var activeJoint: Joint? = null
    private var goal: Vector2 = Vector2()
    private var target: Vector2 = Vector2()

    private val endEffectorD: Double = 3.0
    private val map = mapOf(
        UP to Vector2(0.0, -endEffectorD),
        DOWN to Vector2(0.0, endEffectorD),
        LEFT to Vector2(-endEffectorD, 0.0),
        RIGHT to Vector2(endEffectorD, 0.0)
    )


    init {
        val width = 900f
        val height = 900f

        ref = this
        arm = Arm(
            Pose2d((width / 2).toDouble(), (height / 2).toDouble(), 0.0),
            150.0, 150.0
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

        goal = arm.joints.last().endPose2d.pos

//        arm.joints[0].setAngle(-Math.PI/4)
//        arm.joints[1].setAngle(Math.PI/2)
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
//        (goal + Arm.base.pos).draw(10f)


        if(useIK){
            applyIK()
        } else {
            applyFK()
        }

        // draw f(p)
        val prospectiveEndEffector = ikSolver.getEndEffectorPosition(arm.joints.map{it.position}.toDoubleArray())

        prospectiveEndEffector.localize(Arm.base.pos).draw(20f)


        // draw derivative
        val endEffector = arm.getEndEffector() + Arm.base.pos
        val velocities: DoubleArray = arm.joints.map {it.velocity}.toDoubleArray()
        val dP = ikSolver.getDirectionalDerivative(ikSolver.getTotalDerivative(*arm.getJointPositions()), velocities).times(Joint.maxSpeed/5.0)

        strokeWeight(5f);
        line(endEffector.x.toFloat(), endEffector.y.toFloat(), (endEffector.x + dP[0,0]).toFloat(), (endEffector.y - dP[1,0]).toFloat())

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

    private fun zeroArm(){
        arm.joints.forEach { it.setVelocity(0.0) }
    }

    private fun applyIK(){
        target = (goal - Arm.base.pos)
        if(arm.getEndEffector().dist(target) > 2.0){
            var dx: DoubleArray = try { ikSolver.getIK(target, 5000) } catch (e: IllegalArgumentException) { print(e.message); return }

            for(i in (0..0)){
                break;
                dx = ikSolver.refineSolution(target, dx)
            }
//            print(dx)
            arm.joints.forEachIndexed { i,j -> j.setVelocity(dx[i])}
//            arm.joints.forEachIndexed { i,j -> j.setAngle(j.position + dx[i])}
//            arm.joints.forEachIndexed  { i,j -> j.setAngle(j.position + dx[i])}

        } else {
            arm.joints.forEach { it.setVelocity(0.0)}
        }
    }

    private fun applyFK(){
        activeJoint ?: return
        fill(255f, 0f, 0f)
//        target.draw(10f)
    }

    override fun mousePressed(event: MouseEvent) {
        super.mousePressed(event)
        activeJoint = arm.isOnJointEnd(mouseX, mouseY) ?: return

        if(useIK){
            zeroArm()
        }
        useIK = false;

        mouseDragged(event)
    }

    override fun mouseDragged(event: MouseEvent) {
        super.mouseDragged()
        val j: Joint = activeJoint ?: return;

        val endEffector = Vector2(mouseX.toDouble(), mouseY.toDouble())

        target = endEffector - j.basePose2d.pos

        strokeWeight(0f); fill(255f,0f,0f)

//        var targetPos: Double = -Math.atan(target.x/target.y) + j.basePose2d.theta
        var targetPos = Math.acos(-target.y/target.dist()) * sign(target.x) - j.basePose2d.theta;

//        println(targetPos * 180 / Math.PI)
        j.goToAngle(targetPos)
    }

    override fun mouseReleased() {
        super.mouseReleased()
        activeJoint = null;
    }

    override fun keyPressed() {
        super.keyPressed()
        if(keyCode in map){
            goal += map[keyCode] ?: Vector2.zero
        }

        if(!useIK){
            zeroArm()
        }
        useIK = true
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