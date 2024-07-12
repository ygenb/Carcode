# 1.Compile
cd almost_finished_code_2024_7_1/
catkin_make -DCMAKE_BUILD_TYPE=Release
cd ..
cd LIO-SLAM-main/
catkin_make -DCMAKE_BUILD_TYPE=Release

tips:There may be compilation errors in ROS messages, please try to compile them again multiple times.

# 2.run
//new term
cd almost_finished_code_2024_7_1/
source devel/setup.bash
cd sh_files/
sh init.sh

//new term
cd ../..
cd LIO-SLAM-main/
source devel/setup.bash
sh base_lio.sh 
cd ..

//new term
cd almost_finished_code_2024_7_1/
source devel/setup.bash
cd sh_files/
sh plan.sh
