/**
 * \if English
 * @file Frame.h
 * @brief Frame related function is mainly used to obtain frame data and frame information
 * \else
 * @file Frame.h
 * @brief 帧相关函数，主要用于获取帧数据及帧的信息
 * \endif
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Get the frame index
 *
 * @param[in] frame Frame object
 * @param[out] error Log wrong message
 * @return uint64_t returns the frame index
 * \else
 * @brief 获取帧索引
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧索引
 * \endif
 */
uint64_t ob_frame_index(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the frame format
 *
 * @param[in] frame Frame object
 * @param[out] error  Log error messages
 * @return ob_format returns the frame format
 * \else
 * @brief 获取帧格式
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回帧格式
 * \endif
 */
ob_format ob_frame_format(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the frame type
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return ob_frame_type returns the frame type
 * \else
 * @brief 获取帧类型
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_frame_type 返回帧类型
 * \endif
 */
ob_frame_type ob_frame_get_type(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get frame time stamp (hardware)
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t returns the frame hardware timestamp
 * \else
 * @brief 获取帧硬件时间戳
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧硬件时间戳
 * \endif
 */
uint64_t ob_frame_time_stamp(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get frame time stamp (hardware) us
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t returns the frame hardware timestamp, unit us
 * \else
 * @brief 获取帧硬件时间戳us
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧硬件时间戳，单位us
 * \endif
 */
uint64_t ob_frame_time_stamp_us(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get frame time stamp (system)
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t returns the frame system timestamp
 * \else
 * @brief 获取帧系统时间戳
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧系统时间戳
 * \endif
 */
uint64_t ob_frame_system_time_stamp(ob_frame *frame, ob_error **error); 

/**
 * \if English
 * @brief Get frame data
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return void* * returns frame data pointer
 * \else
 * @brief 获取帧数据
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return void* 返回帧数据指针
 * \endif
 */
void *ob_frame_data(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the frame data size
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint32_t returns the frame data size
 * If it is point cloud data, it returns the number of bytes occupied by all point sets. If you need to find the number of points, you need to divide dataSize by the structure size of the corresponding point type.
 * \else
 * @brief 获取帧数据大小
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧数据大小
 * 如果是点云数据返回的是所有点集合占的字节数，若需要求出点的个数需要将dataSize除以对应的点类型的结构体大小
 * \endif
 */
uint32_t ob_frame_data_size(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get video frame width
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint32_t returns the frame width
 * \else
 * @brief 获取视频帧宽
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧宽
 * \endif
 */
uint32_t ob_video_frame_width(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get video frame height
 *
 * @param[in] frame  Frame object
 * @param[out] error Log error messages
 * @return uint32_t returns the frame height
 * \else
 * @brief 获取视频帧高
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧高
 * \endif
 */
uint32_t ob_video_frame_height(ob_frame *frame, ob_error **error);

/**
 *\if English
 * @brief Get the metadata of the frame
 *
 * @param[in] frame  Video frame object
 * @param[out] error Log error messages
 * @return void* returns the metadata pointer of the frame
 * \else
 * @brief 获取帧的元数据
 *
 * @param[in] frame 视频帧对象
 * @param[out] error 记录错误信息
 * @return void* 返回帧的元数据指针
 * \endif
 */
void *ob_video_frame_metadata(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the metadata size of the frame
 *
 * @param[in] frame Video frame object
 * @param[out] error Log error messages
 * @return uint32_t returns the metadata size of the frame
 * \else
 * @brief 获取帧的元数据大小
 *
 * @param[in] frame 视频帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧的元数据大小
 * \endif
 */
uint32_t ob_video_frame_metadata_size(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief  Get the effective number of pixels (such as Y16 format frame, but only the lower 10 bits are effective bits, and the upper 6 bits are filled with 0)
 * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format
 *
 * @param[in] frame video frame object
 * @param[out] error log error messages
 * @return uint8_t return the effective number of pixels in the pixel, or 0 if it is an unsupported format
 * \else
 * @brief  获取像素有效位数（如Y16格式帧，每个像素占16bit，但实际只有低10位是有效位，高6位填充0）
 * @attention 仅对Y8/Y10/Y11/Y12/Y14/Y16格式有效
 *
 * @param[in] frame 视频帧对象
 * @param[out] error 记录错误信息
 * @return uint8_t 返回像素有效位数，如果是不支持的格式，返回0
 * \endif
 */
uint8_t ob_video_frame_pixel_available_bit_size(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the value scale of the depth frame, the unit is mm/step,
 *      such as valueScale=0.1, and a certain coordinate pixel value is pixelValue=10000,
 *      then the depth value = pixelValue*valueScale = 10000*0.1=1000mm。
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return float value scale
 * \else
 * @brief 获取深度帧的值刻度，单位为 mm/step，
 *      如valueScale=0.1, 某坐标像素值为pixelValue=10000，
 *     则表示深度值value = pixelValue*valueScale = 10000*0.1=1000mm。
 *
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return float 值刻度
 * \endif
 */
float ob_depth_frame_get_value_scale(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Delete frame
 *
 * @param[in] frame  The frame object to delete
 * @param[out] error Log error messages
 * \else
 * @brief 删除帧
 *
 * @param[in] frame 要删除的帧对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_frame(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the number of frames contained in the frameset
 *
 * @param[in] frameset frameset object
 * @param[out] error Log error messages
 * @return uint32_t returns the number of frames
 * \else
 * @brief 获取帧集合包含的帧数量
 *
 * @param[in] frameset 帧集合对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧数量
 * \endif
 */
uint32_t ob_frameset_frame_count(ob_frame *frameset, ob_error **error);

/**
 * \if English
 * @brief Get the depth frame from the frameset
 *
 * @param[in] frameset  Frameset object
 * @param[out] error Log error messages
 * @return ob_frame* returns the depth frame
 * \else
 * @brief 从帧集合中获取深度帧
 *
 * @param[in] frameset 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回深度帧
 * \endif
 */
ob_frame *ob_frameset_depth_frame(ob_frame *frameset, ob_error **error);

/**
 * \if English
 * @brief Get the color frame from the frameset
 *
 * @param[in] frameset Frameset object
 * @param[out] error Log error messages
 * @return ob_frame* returns the color frame
 * \else
 * @brief 从帧集合中获取彩色帧
 *
 * @param[in] frameset 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回彩色帧
 * \endif
 */
ob_frame *ob_frameset_color_frame(ob_frame *frameset, ob_error **error);

/**
 * \if English
 * @brief Get the infrared frame from the frameset
 *
 * @param[in] frameset Frameset object
 * @param[out] error Log error messages
 * @return ob_frame* returns the infrared frame
 * \else
 * @brief 从帧集合中获取红外帧
 *
 * @param[in] frameset 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回红外帧
 * \endif
 */
ob_frame *ob_frameset_ir_frame(ob_frame *frameset, ob_error **error);

/**
 * \if English
 * @brief Get point cloud data from the frameset
 *
 * @param[in] frameset Frameset object
 * @param[out] error Log error messages
 * @return ob_frame* returns the point cloud frame
 * \else
 * @brief 从帧集合中获取点云数据
 *
 * @param[in] frameset 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回点云帧
 * \endif
 */
ob_frame *ob_frameset_points_frame(ob_frame *frameset, ob_error **error);

/**
 * \if English
 * @brief Get accelerometer frame data
 *
 * @param[in] frame Accelerometer frame
 * @param[out] error  Log error messages
 * @return ob_accel_value accelerometer data
 * \else
 * @brief 获取加速度帧数据
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return ob_accel_value 加速度数据
 * \endif
 */
ob_accel_value ob_accel_frame_value(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get the temperature when acquiring the accelerometer frame
 *
 * @param[in] frame Accelerometer frame
 * @param[out] error Log error messages
 * @return float return value
 * \else
 * @brief 获取加速度帧采样时温度
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return float 返回数值
 * \endif
 */
float ob_accel_frame_temperature(ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Get gyroscope frame data
 *
 * @param[in] frame Gyro Frame
 * @param[out] error Log error messages
 * @return ob_gyro_value gyroscope data
 * \else
 * @brief 获取陀螺仪帧数据
 *
 * @param[in] frame 陀螺仪帧
 * @param[out] error 记录错误信息
 * @return ob_gyro_value 陀螺仪数据
 * \endif
 */
ob_gyro_value ob_gyro_frame_value(ob_frame *frame, ob_error **error);

/**
 *\if English
 * @brief Get the temperature when acquiring the gyroscope frame
 *
 * @param[in] frame Accelerometer frame
 * @param[out] error Log error messages
 * @return float return value
 * \else
 * @brief 获取加速度帧采样时温度
 *
 * @param[in] frame 加速度帧
 * @param[out] error 记录错误信息
 * @return float 返回数值
 * \endif
 */
float ob_gyro_frame_temperature(ob_frame *frame, ob_error **error);

#ifdef __cplusplus
}
#endif
