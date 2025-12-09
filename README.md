# mcu_arm_temp
mcu template project

# ccs环境配置
ccs选择CCSWORKSPACE,从Target/TMS320Fxx导入project


# ccs新增芯片
 ccs创建新project,选择路径为target/xxxx
添加文件夹：File->new->Folder->advance->linkFolder
添加头文件：project->build->c2000compiler->include options


# gd32e230配置
选择c_cpp_properties.json, 右下角选择gd32e230, 在includePath头文件选择自己电脑编译器路径

编译方法（windows）：
根目录：

cmake --preset GD32E230 -B Target/GD32E230/build --fresh (推荐每次增加或修改文件或修改核心文件如.s文件后执行)

cmake --build --preset GD32E230构建



旧方法：进入build文件夹执行：cmake -G 'MinGW Makefiles' ..\..\..   再执行mingw32-make

GD32e23x时钟选择：见rcu.h
