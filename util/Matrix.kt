package util

import java.lang.IllegalArgumentException
import java.util.*
import kotlin.math.cos
import kotlin.math.sin

class Matrix constructor(val rows: Int, val cols: Int) {

    // init empty array
    private val vals: Array<Array<Double>> = Array<Array<Double>>(rows) { Array<Double>(cols) {0.0} }

    constructor(rows: Int, cols: Int, init: (Int, Int) -> Double): this(rows, cols) {
        for(r in 0..rows){
            for(c in 0..cols){
                vals[r][c] = init(r,c)
            }
        }
    }

    fun get(r: Int, c: Int): Double{
        return vals[r][c]
    }

    fun set(r: Int, c: Int, value: Double){
        vals[r][c] = value
    }


    operator fun unaryMinus(): Matrix = Matrix(rows, cols) { r:Int, c:Int -> -get(r,c)}

    operator fun plus(other: Matrix): Matrix {
        if(rows != other.rows || cols != other.cols) throw IllegalArgumentException("undefined operation")

        return Matrix(rows, cols) {r:Int,c:Int -> get(r,c) + other.get(r,c)}
    }

    operator fun minus(other: Matrix): Matrix {
        if(rows != other.rows || cols != other.cols) throw IllegalArgumentException("undefined operation")

        return Matrix(rows, cols) {r:Int,c:Int -> get(r,c) - other.get(r,c)}
    }

    operator fun times(value: Double): Matrix = Matrix(rows, cols) { r:Int, c: Int -> value * get(r,c)}

    operator fun times(other: Matrix): Matrix {
        if(rows != other.rows || cols != other.cols)
            throw IllegalArgumentException("undefined operation")

        return Matrix(rows, cols) { r: Int, c: Int ->
            var dot = 0.0
            for(i in 0..rows){
                for(j in 0..cols){
                    dot += get(i,j) * other.get(j,i)
                }
            }
            dot
        }
    }

    fun t() : Matrix = Matrix(cols, rows){ r,c -> get(c,r) }

    inline fun map(crossinline transform: (Int, Int, Double) -> Double): Matrix {
        return Matrix(rows, cols) { r,c -> transform(r,c,get(r,c)) }
    }

    companion object{
        fun rotationMatrix(theta: Double): Matrix {
            val m = Matrix(2,2)
            m.set(0,0, cos(theta))
            m.set(0,1,-sin(theta))
            m.set(1,0, sin(theta))
            m.set(1,1, cos(theta))
            return m
        }

        fun identityMatrix(dim: Int = 2): Matrix = Matrix(dim,dim)
            {r,c -> if (r == c) 1.0 else 0.0}
    }
}