# mcu_arm_temp
mcu template project

# tms320 config(ccs)
ccs选择CCSWORKSPACE,从Target/TMS320Fxx导入project


# ccs新增芯片
 ccs创建新project,选择路径为target/xxxx
添加文件夹：File->new->Folder->advance->linkFolder
添加头文件：project->build->c2000compiler->include options

# gd32e230配置
选择c_cpp_properties.json, 右下角选择gd32e230, 在includePath头文件选择自己电脑编译器路径

编译方法：进入build文件夹执行：cmake -G 'MinGW Makefiles' ..\..\..   再执行mingw32-make