/**
 * @file Frame.h
 * @brief 帧相关函数，主要用于获取帧数据及帧的信息
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取帧索引
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧索引
 */
uint64_t ob_frame_index( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧格式
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回帧格式
 */
ob_format ob_frame_format( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧类型
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_frame_type 返回帧类型
 */
ob_frame_type ob_frame_get_type( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧硬件时间戳
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧硬件时间戳
 */
uint64_t ob_frame_time_stamp( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧系统时间戳
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧系统时间戳
 */
uint64_t ob_frame_system_time_stamp( ob_frame* frame, ob_error** error );

/**
 * @brief 获取视频帧宽
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧宽
 */
uint32_t ob_video_frame_width( ob_frame* frame, ob_error** error );

/**
 * @brief 获取视频帧高
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧高
 */
uint32_t ob_video_frame_height( ob_frame* frame, ob_error** error );

/**
 * @brief 获取视频帧宽 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_video_frame_width代替
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧宽
 */
DEPRECATED uint32_t ob_frame_width( ob_frame* frame, ob_error** error );

/**
 * @brief 获取视频帧高 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_video_frame_height代替
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧高
 */
DEPRECATED uint32_t ob_frame_height( ob_frame* frame, ob_error** error );

/**
 * @brief 获取深度帧的值刻度，单位为 mm/step，
 *      如valueScale=0.1, 某坐标像素值为pixelValue=10000，
 *     则表示深度值value = pixelValue*valueScale = 10000*0.1=1000mm。
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return float 值刻度
 */
float ob_depth_frame_get_value_scale( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧数据
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return void* 返回帧数据指针
 */
void* ob_frame_data( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧数据大小
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧数据大小
 */
uint32_t ob_frame_data_size( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧的元数据
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return void* 返回帧的元数据指针
 */
void* ob_frame_metadata( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧的元数据大小
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧的元数据大小
 */
uint32_t ob_frame_metadata_size( ob_frame* frame, ob_error** error );

/**
 * @brief 获取帧的流类型
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 返回流类型
 */
ob_stream_type ob_frame_get_stream_type( ob_frame* frame, ob_error** error );

/**
 * @brief 删除帧
 *
 * @param[in] frame 要删除的帧对象
 * @param[out] error 记录错误信息
 */
void ob_delete_frame( ob_frame* frame, ob_error** error );

/**
 * @brief 删除帧集合 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_delete_frame
 *
 * @param[in] frame 要删除的帧对象
 * @param[out] error 记录错误信息
 */
DEPRECATED void ob_delete_frame_set( ob_frame* frame_set, ob_error** error );

/**
 * @brief 获取帧集合包含的帧数量
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧数量
 */
uint32_t ob_frame_set_frame_count( ob_frame* frame_set, ob_error** error );

/**
 * @brief 从帧集合中获取深度帧
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回深度帧
 */
ob_frame* ob_frame_set_depth_frame( ob_frame* frame_set, ob_error** error );

/**
 * @brief 从帧集合中获取彩色帧
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回彩色帧
 */
ob_frame* ob_frame_set_color_frame( ob_frame* frame_set, ob_error** error );

/**
 * @brief 从帧集合中获取红外帧
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回红外帧
 */
ob_frame* ob_frame_set_ir_frame( ob_frame* frame_set, ob_error** error );

/**
 * @brief 从帧集合中获取点云数据
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回点云帧
 */
ob_frame* ob_frame_set_points_frame( ob_frame* frame_set, ob_error** error );

/**
 * @brief 获取加速度帧数据
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return ob_accel_value 加速度数据
 */
ob_accel_value ob_accel_frame_value( ob_frame* frame, ob_error** error );

/**
 * @brief 获取加速度帧采样时温度
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return float 返回数值
 */
float ob_accel_frame_temperature( ob_frame* frame, ob_error** error );

/**
 * @brief 获取陀螺仪帧数据
 *
 * @param[in] frame 陀螺仪帧
 * @param[out] error 记录错误信息
 * @return ob_gyro_value 陀螺仪数据
 */
ob_gyro_value ob_gyro_frame_value( ob_frame* frame, ob_error** error );

/**
 * @brief 获取加速度帧采样时温度
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return float 返回数值
 */
float ob_gyro_frame_temperature( ob_frame* frame, ob_error** error );

#ifdef __cplusplus
}
#endif
