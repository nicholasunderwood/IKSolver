import util.Matrix
import util.Vector2
import  Ma

class IKSolver constructor(private val arm: Arm, target: Vector2){

    private val initJointSpace: Matrix = arm.getJointSpace()
    private val initPos: Matrix = arm.getEndEffector(initJointSpace)

    fun constructJacobianMatrix(x: Double, y: Double): ArrayList<Double> {
        val endpoints: ArrayList<Double> = ArrayList<Double>(workspaceDim)

        val jacobianMatrix = Matrix(workspaceDim, arm.dof)

        val d = Matrix(workspaceDim,1)

        for(i in 0..workspaceDim){
            d.set(i,0, epsilon)
            if (i > 0) d.set(i-1, 0, epsilon)
            val dx = initPos + d
            val dp = arm.getEndEffector(dx)

            for(k in 0..arm.dof){
                jacobianMatrix.set(i,k, (dp.get(i,k) - initPos.cols) / epsilon)
            }
        }

        return endpoints
    }

    companion object{
        const val epsilon: Double = 1e-2
        const val workspaceDim: Int = 2

    }

}