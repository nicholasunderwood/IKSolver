package util

class Pose2d constructor(var pos: Vector2 = Vector2(), var theta: Double = 0.0){
    constructor(x: Double, y: Double, theta: Double): this(Vector2(x,y), theta)

    var x: Double
        get() = pos.x
        set(value: Double) { pos.x = value }

    var y: Double
        get() = pos.y
        set(value: Double) { pos.y = value }


    operator fun unaryMinus(): Pose2d = Pose2d(-pos, -theta)

    operator fun plus(other: Pose2d): Pose2d = Pose2d(pos + other.pos, theta + other.theta)

    operator fun minus(other: Pose2d): Pose2d = this + -other

    operator fun times(c: Double): Pose2d = Pose2d(pos*c, theta)

    operator fun plusAssign(other: Pose2d) {
        pos += other.pos
        theta += other.theta
    }

    override fun toString(): String = "($x, $y), $theta"

    fun clone(): Pose2d {
        return Pose2d(x, y, theta)
    }

}