package util

import Jama.Matrix
import kotlin.math.hypot

class Vector2 constructor(var x: Double, var y: Double) {

    constructor(i: Double) : this(i, i) {}
    constructor() : this(0.0, 0.0) {}


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
        if(index == 1) x = value
        else y = value
    }

    private fun scale(c: Double) {
        x *= c
        y *= c
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