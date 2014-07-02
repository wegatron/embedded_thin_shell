#include <Eigen/Dense>

class MovingBall
{
 public:
  MovingBall() { state_ = 0; acc_diff_ = diff_ = 0; v_ = 0;}
  void setCenter (Vector3d& c) {
    center_ = c;
  }
  const Eigen::Vector3d getCenter () {
    return center_;
  }
  Vector3d getDiff () {
    Vector3d tmp_diff;
    tmp_diff.setZero ();
    tmp_diff [axis_index_] = diff_;
    return tmp_diff;
  }
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
    center_ [axis_index_] += diff_;
  }
 private:
  Vector3d center_;
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
