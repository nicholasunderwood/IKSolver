import Jama.Matrix
import Jama.SingularValueDecomposition
import util.Vector2
import java.lang.IllegalArgumentException
import java.lang.RuntimeException

class IKSolver constructor(private val arm: Arm){

	private val f: Array<out (DoubleArray) -> Double> = arrayOf(
		{ p: DoubleArray -> p.indices.sumByDouble { arm.joints[it].length * Math.sin((0..it).sumByDouble { i -> p[i] }) }},
		{ p: DoubleArray -> p.indices.sumByDouble { arm.joints[it].length * Math.cos((0..it).sumByDouble { i -> p[i] }) } }
	)

	private val jac: Array<Array<out (DoubleArray) -> Double>> = arrayOf(
		Array<(DoubleArray) -> Double>(arm.axes){ i -> { p: DoubleArray ->
			(i until p.size).sumByDouble { arm.joints[it].length * Math.cos( (0..it).sumByDouble { k -> p[k] } ) }
		}  },
		Array<(DoubleArray) -> Double>(arm.axes){ i -> { p: DoubleArray ->
			(i until p.size).sumByDouble { arm.joints[it].length * -Math.sin( (0..it).sumByDouble { k -> p[k] } ) }
		}  }
	)

	fun getIK(goal: Vector2, lineSearchIterations: Int, tolarance: Double = 1.0): DoubleArray {
		val x0: DoubleArray = arm.joints.map{it.position}.toDoubleArray() // initial joint position
		val p0: Vector2 = arm.getEndEffector() // initial end effectors
		var dP: Vector2 = goal - p0 // difference in end effectors


		val Jp: Matrix = getTotalDerivative(*x0) // Jacobian matrix dX -> dP
		val Jx = getPseudoInverseSVD(Jp)


		val h = makeColumMatrix((goal-p0).toDoubleArray()) // Column matrix of dP
//		val dx = Matrix.random(3,1).times(Math.PI * 2).minus(Matrix(3,1,Math.PI));
		val dx = Jx.times(h);
//		dx.print(4,2)
//		dThetas.print(4,2)
		return dx.times(1/dx.normF()).columnPackedCopy
	}

	private fun makeColumMatrix(vector: DoubleArray): Matrix {
		return Matrix(Array<DoubleArray>(vector.size) {i -> DoubleArray(1){ vector[i] }} )
	}


	private fun getPseudoInverseSVD(m: Matrix): Matrix {
		val SVDp = SingularValueDecomposition(m)

		val u = Matrix(SVDp.u.array)
		val s = Matrix(SVDp.s.array.copyOfRange(0,u.rowDimension)) // workaround for bug in JAMA SVD
		val v = SVDp.v

		val sArray = s.arrayCopy
		for(i in  (0 until Math.min(s.columnDimension, s.rowDimension))){
			sArray[i][i] = if (sArray[i][i] == 0.0) 0.0 else 1/sArray[i][i]
		}
		val sInverse: Matrix = Matrix(sArray).transpose()

		return try{ v.transpose() * sInverse * u.transpose() } catch (e: RuntimeException){  // Inverse Jacobian Matrix dP -> dX
			throw IllegalArgumentException(e.message)
		}
	}

	private fun getPseduoInverseDef(m: Matrix): Matrix {
		return (m*m.transpose()).inverse() * m.transpose()
	}

	fun refineSolution(goal: Vector2, initialGuess: DoubleArray): DoubleArray{
		val x0: DoubleArray = arm.joints.map{it.position}.toDoubleArray() // initial joint position

		val dx = makeColumMatrix(initialGuess)
		val xk = makeColumMatrix(x0) + dx
		val Jp = getTotalDerivative(*xk.columnPackedCopy)
		val Jx = getPseudoInverseSVD(Jp)


		val dP = arm.getEndEffector((dx + makeColumMatrix(x0)).columnPackedCopy)
		val h = Matrix(Array<DoubleArray>(2) {i -> DoubleArray(1){ dP[i] }} ) // Column matrix of dP
		return Jx.times(h).columnPackedCopy;
	}

	fun getEndEffectorPosition(p: DoubleArray) = Vector2(f[0](p), f[1](p))

	fun getTotalDerivative(vararg jointConfig: Double) : Matrix{
		val ar: Array<DoubleArray> = arrayOf(
			DoubleArray(arm.axes) { jac[0][it](jointConfig) },
			DoubleArray(arm.axes) { jac[1][it](jointConfig) }
		)
		return Matrix(ar)
	}

	fun getDirectionalDerivative(totalDerivative: Matrix, direction: DoubleArray): Matrix {
		return totalDerivative * makeColumMatrix(direction)
	}

	private val sin: (Double) -> Double = {x: Double -> Math.sin(x)}
	private val cos: (Double) -> Double = {x: Double -> Math.cos(x)}

	companion object{
		const val epsilon: Double = 1e-4
		const val workspaceDim: Int = 2
		const val numJoints: Int = 3
	}

}