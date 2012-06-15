# CMakeLists for STM32F4 using the Sourcery toolchain and OpenOCD with both
# C and C++ sources
# 
# To adapt this for any other STM32 chip should be trivial
# To adapt to any other ARM controller shouldn't be much worse
#
# A few convenience targets are provided for OpenOCD, GDB, and Ctags
#
# The structure is as follows
#  /CMSIS
#  /STM32F4xx_StdPeriph...
#  /src (application code)
#  /src/system (application code - low-level configuration)
#  /stm32_flash.ld (linker script for !!!STM32F4 only!!! from Atollic TrueStudio)
#
# Author: Ben Nahill <bnahill@gmail.com>
#


# Use the Sourcery ARM EABI toolchain
set(TC_PREFIX arm-none-eabi-)
set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)

set(TARGET xnemo)
set(OUTFILE ${TARGET}.elf)

project(${TARGET})
cmake_minimum_required(VERSION 2.8)

# Path to the STM32F4 Peripheral Library driver directory
set(STD_PERIPH_DIR STM32F4xx_StdPeriph_Driver)
# Path to CMSIS
set(CMSIS_DIR CMSIS)
set(MCU cortex-m4)


set(MCFLAGS "-mcpu=${MCU} -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

add_definitions(${MCFLAGS})
add_definitions(-g -ffunction-sections  -fdata-sections -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align)
add_definitions(-DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DHSE_VALUE=8000000 -DARM_MATH_CM4 -D__FPU_PRESENT)

# Eliminate the need for a bunch of extra support code that may not work on the chip anyways
set(CMAKE_CXX_FLAGS "--std=c++0x -fno-rtti -fno-exceptions")
# Use the new C11 standard
set(CMAKE_C_FLAGS "--std=c1x")
set(CMAKE_EXE_LINKER_FLAGS "-Tlink.ld -flto  -Wl,-Map=${TARGET}.map,--cref,--gc-sections -nostdlib ${MCFLAGS} ${CMAKE_EXE_LINKER_FLAGS}")

set(STD_PERIPH_SRC_DIR ${STD_PERIPH_DIR}/src)

set(STM_SOURCES
	${STD_PERIPH_SRC_DIR}/misc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_dac.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_hash.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_rtc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_adc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_dbgmcu.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_hash_md5.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_sdio.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_can.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_dcmi.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_hash_sha1.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_spi.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_crc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_dma.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_i2c.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_syscfg.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_cryp_aes.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_exti.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_iwdg.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_tim.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_cryp.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_flash.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_pwr.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_usart.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_cryp_des.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_fsmc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_rcc.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_wwdg.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_cryp_tdes.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_gpio.c
	${STD_PERIPH_SRC_DIR}/stm32f4xx_rng.c
)

set(DSP_DIR ${CMSIS_DIR}/DSP_Lib/Source)

set(CMSIS_DSP_SOURCES
	${DSP_DIR}/SupportFunctions/arm_fill_q7.c
	${DSP_DIR}/SupportFunctions/arm_q7_to_float.c
	${DSP_DIR}/SupportFunctions/arm_q7_to_q31.c
	${DSP_DIR}/SupportFunctions/arm_float_to_q31.c
	${DSP_DIR}/SupportFunctions/arm_q7_to_q15.c
	${DSP_DIR}/SupportFunctions/arm_q31_to_q7.c
	${DSP_DIR}/SupportFunctions/arm_q15_to_q31.c
	${DSP_DIR}/SupportFunctions/arm_q15_to_float.c
	${DSP_DIR}/SupportFunctions/arm_copy_q15.c
	${DSP_DIR}/SupportFunctions/arm_copy_q7.c
	${DSP_DIR}/SupportFunctions/arm_q31_to_float.c
	${DSP_DIR}/SupportFunctions/arm_float_to_q7.c
	${DSP_DIR}/SupportFunctions/arm_q15_to_q7.c
	${DSP_DIR}/SupportFunctions/arm_fill_f32.c
	${DSP_DIR}/SupportFunctions/arm_copy_q31.c
	${DSP_DIR}/SupportFunctions/arm_q31_to_q15.c
	${DSP_DIR}/SupportFunctions/arm_copy_f32.c
	${DSP_DIR}/SupportFunctions/arm_fill_q31.c
	${DSP_DIR}/SupportFunctions/arm_float_to_q15.c
	${DSP_DIR}/SupportFunctions/arm_fill_q15.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_f32.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_lms_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_q7.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df2T_f32.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_f32.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_f32.c
	${DSP_DIR}/FilteringFunctions/arm_conv_q7.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_f32.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_q15.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_lms_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_q7.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_q7.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_init_q7.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_q31.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_f32.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_32x64_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_init_q7.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_f32.c
	${DSP_DIR}/FilteringFunctions/arm_conv_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_init_q31.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_q31.c
	${DSP_DIR}/FilteringFunctions/arm_lms_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_interpolate_q31.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_q15.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_q31.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_q31.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_q7.c
	${DSP_DIR}/FilteringFunctions/arm_lms_q15.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_q31.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_q31.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_q15.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_init_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_f32.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_biquad_cascade_df1_32x64_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_f32.c
	${DSP_DIR}/FilteringFunctions/arm_lms_norm_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_lattice_q15.c
	${DSP_DIR}/FilteringFunctions/arm_lms_q31.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_q15.c
	${DSP_DIR}/FilteringFunctions/arm_correlate_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_sparse_f32.c
	${DSP_DIR}/FilteringFunctions/arm_iir_lattice_init_q15.c
	${DSP_DIR}/FilteringFunctions/arm_fir_decimate_fast_q31.c
	${DSP_DIR}/FilteringFunctions/arm_conv_partial_fast_q15.c
	${DSP_DIR}/FilteringFunctions/arm_lms_init_q31.c
	${DSP_DIR}/CommonTables/arm_common_tables.c
	${DSP_DIR}/MatrixFunctions/arm_mat_mult_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_mult_fast_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_add_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_init_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_add_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_init_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_trans_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_init_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_trans_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_scale_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_add_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_mult_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_mult_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_sub_q31.c
	${DSP_DIR}/MatrixFunctions/arm_mat_scale_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_inverse_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_mult_fast_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_sub_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_sub_q15.c
	${DSP_DIR}/MatrixFunctions/arm_mat_scale_f32.c
	${DSP_DIR}/MatrixFunctions/arm_mat_trans_q15.c
	${DSP_DIR}/FastMathFunctions/arm_cos_f32.c
	${DSP_DIR}/FastMathFunctions/arm_cos_q15.c
	${DSP_DIR}/FastMathFunctions/arm_sqrt_q31.c
	${DSP_DIR}/FastMathFunctions/arm_cos_q31.c
	${DSP_DIR}/FastMathFunctions/arm_sin_f32.c
	${DSP_DIR}/FastMathFunctions/arm_sin_q15.c
	${DSP_DIR}/FastMathFunctions/arm_sqrt_q15.c
	${DSP_DIR}/FastMathFunctions/arm_sin_q31.c
	${DSP_DIR}/TransformFunctions/arm_rfft_q15.c
	${DSP_DIR}/TransformFunctions/arm_rfft_init_q31.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_init_q15.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_q15.c
	${DSP_DIR}/TransformFunctions/arm_dct4_init_q15.c
	${DSP_DIR}/TransformFunctions/arm_rfft_init_f32.c
	${DSP_DIR}/TransformFunctions/arm_dct4_q31.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_init_q31.c
	${DSP_DIR}/TransformFunctions/arm_rfft_f32.c
	${DSP_DIR}/TransformFunctions/arm_dct4_q15.c
	${DSP_DIR}/TransformFunctions/arm_dct4_f32.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_init_f32.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_q31.c
	${DSP_DIR}/TransformFunctions/arm_dct4_init_f32.c
	${DSP_DIR}/TransformFunctions/arm_rfft_init_q15.c
	${DSP_DIR}/TransformFunctions/arm_dct4_init_q31.c
	${DSP_DIR}/TransformFunctions/arm_rfft_q31.c
	${DSP_DIR}/TransformFunctions/arm_cfft_radix4_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_scale_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_abs_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_abs_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_shift_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_dot_prod_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_negate_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_scale_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_mult_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_dot_prod_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_offset_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_add_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_abs_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_scale_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_shift_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_sub_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_add_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_mult_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_add_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_abs_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_sub_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_negate_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_offset_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_shift_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_offset_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_add_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_dot_prod_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_negate_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_negate_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_dot_prod_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_scale_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_offset_f32.c
	${DSP_DIR}/BasicMathFunctions/arm_sub_q15.c
	${DSP_DIR}/BasicMathFunctions/arm_sub_q31.c
	${DSP_DIR}/BasicMathFunctions/arm_mult_q7.c
	${DSP_DIR}/BasicMathFunctions/arm_mult_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_f32.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_conj_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_real_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_real_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_conj_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_conj_f32.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mult_real_f32.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c
	${DSP_DIR}/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c
	${DSP_DIR}/ControllerFunctions/arm_pid_init_q15.c
	${DSP_DIR}/ControllerFunctions/arm_pid_init_q31.c
	${DSP_DIR}/ControllerFunctions/arm_sin_cos_f32.c
	${DSP_DIR}/ControllerFunctions/arm_pid_reset_q15.c
	${DSP_DIR}/ControllerFunctions/arm_sin_cos_q31.c
	${DSP_DIR}/ControllerFunctions/arm_pid_init_f32.c
	${DSP_DIR}/ControllerFunctions/arm_pid_reset_q31.c
	${DSP_DIR}/ControllerFunctions/arm_pid_reset_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_mean_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_min_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_var_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_power_q7.c
	${DSP_DIR}/StatisticsFunctions/arm_max_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_rms_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_mean_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_mean_q7.c
	${DSP_DIR}/StatisticsFunctions/arm_max_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_min_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_max_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_std_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_power_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_var_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_std_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_power_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_std_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_mean_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_rms_q31.c
	${DSP_DIR}/StatisticsFunctions/arm_min_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_min_q7.c
	${DSP_DIR}/StatisticsFunctions/arm_power_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_var_f32.c
	${DSP_DIR}/StatisticsFunctions/arm_rms_q15.c
	${DSP_DIR}/StatisticsFunctions/arm_max_q7.c
)

set(IMU_DIR imu_lib)

set(IMU_SOURCES
    ${IMU_DIR}/i2c.c
    ${IMU_DIR}/spi.c
    ${IMU_DIR}/l3gd20.c
    ${IMU_DIR}/lsm303.c
    ${IMU_DIR}/sdio.c
    ${IMU_DIR}/sdio_ll.c
)


set(APP_DIR src)
set(SYSTEM_DIR ${APP_DIR}/system)

set(SYSTEM_SOURCES
	${SYSTEM_DIR}/stm32f4xx_it.c
	${SYSTEM_DIR}/system_stm32f4xx.c
	${SYSTEM_DIR}/startup_stm32f4xx.s
)

# Please don't just ignore that assembly file...
set_property(SOURCE ${SYSTEM_DIR}/startup_stm32f4xx.s PROPERTY LANGUAGE C)

set(APP_SOURCES
	${APP_DIR}/main.c
	${APP_DIR}/tick.c
)


##############################################################
# Executable and library targets
##############################################################

include_directories(
	.
	src
	src/system
	${CMSIS_DIR}/Include
	${CMSIS_DIR}/Device/ST/STM32F4xx/Include
	Device/ARM/ARMCM4/Include
	${STD_PERIPH_DIR}/inc
	${IMU_DIR}
)

add_library(imu_lib STATIC ${IMU_SOURCES})
add_library(std_periph STATIC ${STM_SOURCES})
add_library(cmsis_dsp STATIC ${CMSIS_DSP_SOURCES})
target_link_libraries(imu_lib std_periph)
add_executable(${OUTFILE} ${APP_SOURCES} ${SYSTEM_SOURCES})
target_link_libraries(${OUTFILE} std_periph cmsis_dsp imu_lib)

# Funky workaround to make target dependent on linker script
add_custom_command(OUTPUT link.ld COMMAND cat ${CMAKE_SOURCE_DIR}/stm32_flash.ld > link.ld DEPENDS ${CMAKE_SOURCE_DIR}/stm32_flash.ld)
add_custom_target(linker_script DEPENDS link.ld)
add_dependencies(${OUTFILE} linker_script)

# Warnings only for application code -- Optimizations only on ST/ARM code
set_target_properties(${OUTFILE} PROPERTIES COMPILE_FLAGS "-W -O0")
set_target_properties(std_periph PROPERTIES COMPILE_FLAGS "-w -O0")
set_target_properties(cmsis_dsp PROPERTIES COMPILE_FLAGS "-w -O0")
##############################################################
# Ctags
##############################################################

set(CTAG_OPTS -R --c-kinds=+p --fields=+iaS --extra=+q)
set(CTAG_SEARCH_LIST
	${STD_PERIPH_DIR}/inc
	${CMSIS_DIR}/Device/ST/STM32F4xx/Include
	${CMSIS_DIR}/Include/arm_math.h
	${CMSIS_DIR}/Include/arm_common_tables.h
	${CMSIS_DIR}/Include/core_cm4.h
	${CMSIS_DIR}/Include/core_cm4_simd.h
	${CMSIS_DIR}/Include/core_cmFunc.h
	${CMSIS_DIR}/Include/core_cmInstr.h
)

foreach(p ${CTAG_SEARCH_LIST})
	set(CTAG_SEARCH ${CTAG_SEARCH} ${CMAKE_SOURCE_DIR}/${p})
endforeach(p)

add_custom_target(ctags
	ctags ${CTAG_OPTS} -f ${CMAKE_SOURCE_DIR}/src/ext_tags ${CTAG_SEARCH}
	COMMENT "Generating Ctags..."
)

##############################################################
# Configuration for GDB
##############################################################

set(GDB    ${TC_PREFIX}gdb)
set(GDBTUI ${TC_PREFIX}gdbtui)

set(GDB_ARGS -ex "target remote localhost:3333" -ex "mon reset init")
add_custom_target(gdb COMMAND ${GDB} ${GDB_ARGS} ${OUTFILE})
add_custom_target(gdbtui COMMAND ${GDBTUI} ${GDB_ARGS} ${OUTFILE})
add_custom_target(ddd COMMAND ddd --gdb --debugger \"${GDB} ${GDB_ARGS}\" ${OUTFILE} > /dev/null 2>&1 &
	COMMENT "Launching DDD"
)

##############################################################
# Configuration for OpenOCD
##############################################################

set(OPENOCD openocd)
set(OPENOCD_SCRIPTS
	board/stm32f4discovery.cfg
)
foreach(s ${OPENOCD_SCRIPTS})
	set(OPENOCD_ARGS ${OPENOCD_ARGS} -f ${s})
endforeach(s)

add_custom_target(openocd COMMAND ${OPENOCD} ${OPENOCD_ARGS}
	COMMENT "Launching OpenOCD..."
)

##############################################################
# Doxygen Output
##############################################################

find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating documentation with Doxygen" VERBATIM
	)
endif(DOXYGEN_FOUND)

#############################################################
# HEX file generation
#############################################################

add_custom_target(${TARGET}.hex ALL
	COMMAND ${TC_PREFIX}objcopy -O ihex ${OUTFILE} ${TARGET}.hex
	DEPENDS ${OUTFILE}
	COMMENT "Generating Intel HEX output..."
)
