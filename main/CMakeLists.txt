# ============================================================
# 分区：main 组件源文件注册
# 作用：将当前目录所有源文件加入编译，并包含当前目录头文件
# ============================================================
idf_component_register(
    SRCS "main.c" 
         "motor_chassis.c" 
         "as201.c" 
         "chassis_pid.c" 
         "path_planner.c" 
         "screen.c" 
         "config.c" 
         "limit_switch.c" 
         "motor_independent.c" 
         "servo.c" 
         "qr_reader.c"
         "chassis_debug.c"
         "wireless.c"
    INCLUDE_DIRS "."
    # 显式声明依赖后，main 不再自动依赖全部组件，故需列出用到的组件
    PRIV_REQUIRES wifi_ota esp_hosted esp_netif esp_driver_gpio esp_driver_ledc
                  esp_driver_uart esp_timer heap log
)