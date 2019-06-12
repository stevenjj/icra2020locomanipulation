#include <avatar_locomanipulation/feasibility/feasibility.hpp>

feasbility::InverseKinematics(maxsteps){
  for(int ii=0; ii<maxsteps; i++){
    valkyrie.updateFullKinematics(q_end);
    valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);

    ComputeTransError(rfoot_des_pos, rfoot_cur_pos, rfoot_pos_error);
    ComputeQuatError(rfoot_des_quat, rfoot_cur_ori, rfoot_ori_error);
    lfoot_pos_error = lfoot_des_pos - lfoot_cur_pos[2];
    CoM_z_error = CoM_des_z - CoM_cur_pos[2];

    task_error.head<3>() = rfoot_pos_error;
    task_error.segment<3>(3) = rfoot_ori_error;
    task_error.segment<1>(6) = lfoot_pos_error;
    task_error.segment<1>(7) = CoM_z_error;

    ik_error_norm = task_error.norm();

    valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
    valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);
    valkyrie.computeCoMJacobian();

    J_task.topRows(6) = J_rfoot;
    Jtask.block()

  }
}

void feasibility::ComputeTransError(const Eigen::Vector3d & des, const Eigen::Vector3d & current, Eigen::Vector3d & error){
  error = des - current;
}

void feasibility::ComputeQuatError(const Eigen::Quaternion<double> & des,
    							const Eigen::Quaternion<double> & current,
    							Eigen::Vector3d & error){
                    axis_angle = des*current.inverse();
                    error = axis_angle.axis() * axis_angle.angle();
                  }
