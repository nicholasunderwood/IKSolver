import util.Vector2
import Jama.Matrix

class IKSolver constructor(private val arm: Arm, target: Vector2){

    private val initJointSpace: Matrix = arm.getJointSpace()

    private val jac: Array<Array<out (Double) -> Double>> = arrayOf(
        Array<(Double) -> Double>(arm.axes){ i -> {p -> (0..i).sumByDouble { arm.joints[it].length } * Math.cos(p) }  },
        Array<(Double) -> Double>(arm.axes){ i -> {p -> (0..i).sumByDouble { arm.joints[it].length } * Math.sin(p) }  }
    )

    init {

    }

    fun getIK(endEffector: DoubleArray, arm: Arm): DoubleArray {
        val jointConfig: DoubleArray = arm.joints.map{it.position}.toDoubleArray()
        val Dp: Matrix = getTotalDerivative(*jointConfig)
        val Dq: Matrix = Dp.inverse()
        val h = Matrix(Array<DoubleArray>(endEffector.size) {i -> DoubleArray(1){endEffector[i]}} )
        val dThetas = Dp * h
        return dThetas.columnPackedCopy
    }

    private fun getTotalDerivative(vararg jointConfig: Double) : Matrix{
        val ar: Array<DoubleArray> = arrayOf(
            DoubleArray(arm.axes) { jac[0][it](jointConfig[it]) },
            DoubleArray(arm.axes) { jac[1][it](jointConfig[it]) }
        )
        return Matrix(ar)
    }

    private val sin: (Double) -> Double = {x: Double -> Math.sin(x)}
    private val cos: (Double) -> Double = {x: Double -> Math.cos(x)}

    companion object{
        const val epsilon: Double = 1e-2
        const val workspaceDim: Int = 2
        const val numJoints: Int = 3
    }

}