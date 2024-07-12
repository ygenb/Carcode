#ifndef _CORE_COMMON_INC_BASICS_SEMANTICS_H_
#define _CORE_COMMON_INC_BASICS_SEMANTICS_H_

class VehicleParam {
 public:
  inline double width() const { return width_; }
  inline double length() const { return length_; }
  inline double wheel_base() const { return wheel_base_; }
  inline double front_suspension() const { return front_suspension_; }
  inline double rear_suspension() const { return rear_suspension_; }
  inline double max_steering_angle() const { return max_steering_angle_; }
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  inline double d_cr() const { return d_cr_; }

  inline void set_width(const double val) { width_ = val; }
  inline void set_length(const double val) { length_ = val; }
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  inline void set_d_cr(const double val) { d_cr_ = val; }

  /**
   * @brief Print info
   */
  inline void print() const
  {
    printf("VehicleParam:\n");
    printf(" -- width:\t %lf.\n", width_);
    printf(" -- length:\t %lf.\n", length_);
    printf(" -- wheel_base:\t %lf.\n", wheel_base_);
    printf(" -- front_suspension:\t %lf.\n", front_suspension_);
    printf(" -- rear_suspension:\t %lf.\n", rear_suspension_);
    printf(" -- d_cr:\t %lf.\n", d_cr_);
    printf(" -- max_steering_angle:\t %lf.\n", max_steering_angle_);
    printf(" -- max_longitudinal_acc:\t %lf.\n", max_longitudinal_acc_);
    printf(" -- max_lateral_acc:\t %lf.\n", max_lateral_acc_);
  }

 private:
  // @zhiwei: agile hunter se
  double width_ = 0.644;
  double length_ = 0.81735;
  double wheel_base_ = 0.65;
  double front_suspension_ = 0.93;  // don't know, no use
  double rear_suspension_ = 1.10;   // don't know, no use
  double max_steering_angle_ = 0.58*180/M_PI;

  double max_longitudinal_acc_ = 2.0;   // don't know, no use
  double max_lateral_acc_ = 2.0;   // don't know, no use

  double d_cr_ = 0.30427;  // length between geometry center and rear axle
};

#endif  // _CORE_COMMON_INC_BASICS_SEMANTICS_H_