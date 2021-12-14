package util

import Jama.Matrix
import kotlin.math.hypot

class Vector2 constructor(x: Double, y: Double) {

    private val vals: DoubleArray = doubleArrayOf(x, y)

    var x: Double
        get() = vals[0]
        set(value: Double) { vals[0] = value }

    var y: Double
            get() = vals[1]
            set(value: Double) { vals[1] = value }


    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())
    constructor(i: Double) : this(i, i)
    constructor() : this(0.0, 0.0)
    constructor(m: DoubleArray) : this(m[0], m[1])
    constructor(m: Matrix):this( if(m.columnDimension > m.rowDimension) m.rowPackedCopy else m.columnPackedCopy  )


//    operator fun set(x: Double, y: Double) {
//        setX(x)
//        setY(y)
//    }

    operator fun get(index: Int): Double = if(index == 0) x else y

    operator fun unaryMinus(): Vector2 = Vector2(-x, -y)

    operator fun plus(other: Vector2) : Vector2 = Vector2(other.x + x, other.y + y)

    operator fun minus(other: Vector2) = plus(-other)

    operator fun times(c: Double) = Vector2(x*c, y*c)

    operator fun timesAssign(c: Double) {scale(c)}

    operator fun set(index: Int, value: Double){
        vals[index] = value
    }

    private fun scale(c: Double) {
        x *= c
        y *= c
    }

    fun unit(): Vector2 {
        return this * (1.0/dist())
    }

    fun toDoubleArray(): DoubleArray{
        return vals.copyOf();
    }

    fun add(other: Vector2) {
        x += other.x
        y += other.y
    }

    fun comb(other: Vector2): Vector2 {
        return Vector2(x + other.x, y + other.y)
    }

    fun dist(other: Vector2): Double {
        return Math.hypot(x - other.x, y - other.y)
    }

    fun dot(other: Vector2): Double {
        return this.x * other.x + this.y * other.y
    }

    fun dist(): Double {
        return Math.hypot(x, y)
    }

    fun norm(): Vector2 {
        return this * (1.0/dist());
    }

    fun draw(extent: Float){
        App.ref.circle(this.x.toFloat(), this.y.toFloat(), extent);
    }

    fun toMatrix(): Matrix {
        return Matrix(2,1,0.0)
    }

    fun norm(other: Vector2): Vector2 {
        return this * (1.0/dist());
    }

    fun clone(): Vector2 {
        return Vector2(x, y)
    }

    override fun toString(): String {
        return "($x, $y)"
    }

    companion object{
        val i = Vector2(1.0,0.0)
        val j = Vector2(0.0,1.0)
    }
}