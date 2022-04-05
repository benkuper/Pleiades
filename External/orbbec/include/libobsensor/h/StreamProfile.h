/**
 * @file StreamProfile.h
 * @brief 流配置相关函数，用于获取流的宽、高、帧率及格式等信息
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取流配置的格式
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回流的格式
 */
ob_format ob_stream_profile_format( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取流的类型
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 流的类型
 */
ob_stream_type ob_stream_profile_type( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的帧率 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_video_stream_profile_fps
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的帧率
 */
DEPRECATED uint32_t ob_stream_profile_fps( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的宽 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_video_stream_profile_width
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的宽
 */
DEPRECATED uint32_t ob_stream_profile_width( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的高 -> 接口已弃用，将会在接下来的版本更新中删除，请使用ob_video_stream_profile_height
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的高
 */
DEPRECATED uint32_t ob_stream_profile_height( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的帧率
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的帧率
 */
uint32_t ob_video_stream_profile_fps( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的宽
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的宽
 */
uint32_t ob_video_stream_profile_width( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取视频流配置的高
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的高
 */
uint32_t ob_video_stream_profile_height( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取加速度计流配置的量程范围
 *
 * @param[in] profile 流配置对象, 如果该配置不是加速度计流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_accel_full_scale_range 量程范围
 */
ob_accel_full_scale_range ob_accel_stream_profile_full_scale_range( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取加速度计流配置的采样频率
 *
 * @param[in] profile 流配置对象, 如果该配置不是加速度计流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_accel_sample_rate 采样频率
 */
ob_accel_sample_rate ob_accel_stream_profile_sample_rate( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取陀螺仪流配置的量程范围
 *
 * @param[in] profile 流配置对象, 如果该配置不是陀螺仪流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_gyro_full_scale_range 量程范围
 */
ob_gyro_full_scale_range ob_gyro_stream_profile_full_scale_range( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取陀螺仪流配置的采样频率
 *
 * @param[in] profile 流配置对象, 如果该配置不是陀螺仪流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_gyro_sample_rate 采样频率
 */
ob_gyro_sample_rate ob_gyro_stream_profile_sample_rate( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 删除流配置列表
 *
 * @param[in] profiles 流配置列表
 * @param[in] count 流配置数量
 * @param[out] error 记录错误信息
 */
void ob_delete_stream_profiles( ob_stream_profile** profiles, uint32_t count, ob_error** error );

/**
 * @brief 删除流配置
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 */
void ob_delete_stream_profile( ob_stream_profile* profile, ob_error** error );

#ifdef __cplusplus
}
#endif