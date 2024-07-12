#include <vehicle_params/vehicle_param.h>

void VehicleParam::print()
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