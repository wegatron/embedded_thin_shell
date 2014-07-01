#include <sstream>
#include <Objmesh.h>
#include <Eigen/Dense>

using namespace Eigen;

void move_obj(UTILITY::Objmesh &obj, Vector3d &diff) {
  VectorXd &verts = obj.getModifyVerts();
  for (int i=0; i<verts.size()/3; ++i) {
    verts.segment<3>(i*3) += diff;
  }
}

void print_center(UTILITY::Objmesh &obj) {
  VectorXd &verts = obj.getModifyVerts();
  int num = verts.size()/3;
  Vector3d sum;
  sum.setZero();
  for (int i=0; i<num; ++i) {
    sum += verts.segment<3>(i*3);
  }

  cout << "center:" << (1.0/num)*sum.transpose() << endl;
}
class MovingBall
{
public:
  MovingBall() { state_ = 0; acc_diff_ = diff_ = 0; v_ = 0;}
  void setBounceV(double bounce_v) {
    bounce_v_ = bounce_v;
  }
  void setDeltaV(double delta_v) {
    delta_v_ = delta_v;
  }
  void setDecDiff(double dec_diff) {
    dec_diff_ = dec_diff;
  }
  void setAxisIndex(int axis_index) {
    axis_index_ = axis_index;
  }
  void setV(double v) {
    v_ = v;
  }
  void setBounceDiff(double bounce_diff) {
    bounce_diff_ = bounce_diff;
  }
  void stepForward(double time_step) {
    switch (state_) {
    case 0:
      diff_ = time_step*v_;
      if ( fabs(acc_diff_) > dec_diff_) {
        state_=1;
      }
      break;
    case 1:
      v_ += time_step*delta_v_;
      diff_ = time_step*v_;
      if (fabs(acc_diff_) > bounce_diff_) {
        state_=2;
      }
      break;
    case 2:
      diff_ = time_step*bounce_v_;
      break;
    }
    acc_diff_ += diff_;
  }
  double getDiff() {
    return diff_;
  }
private:
  double acc_diff_;
  double diff_;
  double bounce_v_;
  double v_;
  double delta_v_;
  double dec_diff_; // if fabs(diff
  double bounce_diff_;
  int axis_index_;
  int state_; // 0 moving 1:starting decrease velocity; 2:bounce with fix velocity
};

int main(int argc, char *argv[])
{
  UTILITY::Objmesh obj;
  obj.load("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/for_test_drop/hit_ball/model/ball.obj");
  string out_file = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/for_test_drop/hit_ball/result/ball";
  double time_step = 0.1;
  MovingBall ball;
  ball.setBounceV(0.1);
  ball.setDeltaV(0.01);
  ball.setDecDiff(0.38);
  ball.setBounceDiff(0.5);
  ball.setAxisIndex(2);
  ball.setV(-0.1);
  Vector3d move_diff;
  move_diff.setZero();
  for (int i=0; i<147; ++i) {
    stringstream ss;
    ss << out_file << i << ".vtk";
    ball.stepForward(time_step);
    move_diff[2] = ball.getDiff();
    move_obj(obj, move_diff);
    obj.writeVTK(ss.str());
  }
  return 0;
}
