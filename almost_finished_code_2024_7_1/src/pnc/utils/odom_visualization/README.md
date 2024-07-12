# odom_visualization
Prettified visualization of F250 in rviz.

如果想要可视化其他型号的飞机,通用步骤为:
1. 在solidworks等软件中建模
2. 导出stl文件
3. 在blender软件中导入stl,并在XYZ上缩小1000倍
4. (可选)在blender的shading选项卡中对模型进行上色
5. 导出dae,将dae拖到mesh文件夹下
6. 将src/odom_visualization.cpp中的"f250.dae"字段替换成你的dae文件即可
