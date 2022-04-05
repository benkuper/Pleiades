/**
 * @file StreamProfile.hpp
 * @brief 流配置相关类型，用于获取流的宽、高、帧率及格式等信息
 *
 */
#pragma once

#include "Types.hpp"

#include <memory>

struct StreamProfileImpl;
struct StreamProfileListImpl;

namespace ob {

class VideoStreamProfile;
class GyroStreamProfile;
class AccelStreamProfile;
class Config;

class OB_EXTENSION_API StreamProfile : public std::enable_shared_from_this< StreamProfile > {
protected:
    std::unique_ptr< StreamProfileImpl > impl_;

public:
    StreamProfile( std::unique_ptr< StreamProfileImpl > impl );
    StreamProfile( StreamProfile& streamProfile );
    ~StreamProfile();

    /**
     * @brief 获取流的格式
     *
     * @return OBFormat 返回流的格式
     */
    OBFormat format();
    /**
     * @brief 获取流的类型
     *
     * @return OBStreamType 返回流的类型
     */
    OBStreamType type();

    /**
     * @brief 检查帧对象的运行时类型是否与给定类型兼容
     *
     * @tparam T 给定的类型
     * @return bool 返回结果
     */
    template < typename T > bool is();

    /**
     * @brief 对象类型转换
     *
     * @tparam T 目标类型
     * @return std::shared_ptr<T> 返回结果, 如果不能够转换，将抛异常
     */
    template < typename T > std::shared_ptr< T > as() {
        if ( !is< T >() )
            throw "unsupported operation, object's type is not require type";

        return std::static_pointer_cast< T >( std::const_pointer_cast< StreamProfile >( shared_from_this() ) );
    }

    friend class Sensor;
    friend class Config;
};

class OB_EXTENSION_API VideoStreamProfile : public StreamProfile {
public:
    VideoStreamProfile( StreamProfile& profile );
    ~VideoStreamProfile();

    /**
     * @brief 获取流的帧率
     *
     * @return uint32_t 返回流的帧率
     */
    uint32_t fps();
    /**
     * @brief 获取流的宽
     *
     * @return uint32_t 返回流的宽
     */
    uint32_t width();
    /**
     * @brief 获取流的高
     *
     * @return uint32_t 返回流的高
     */
    uint32_t height();
};

class OB_EXTENSION_API AccelStreamProfile : public StreamProfile {
public:
    AccelStreamProfile( StreamProfile& profile );
    ~AccelStreamProfile();

    /**
     * @brief 获取满量程范围
     *
     * @return OBAccelFullScaleRange  返回量程范围值
     */
    OBAccelFullScaleRange fullScaleRange();

    /**
     * @brief 获取采样频率
     *
     * @return OBAccelFullScaleRange  返回采样频率
     */
    OBAccelSampleRate sampleRate();
};

class OB_EXTENSION_API GyroStreamProfile : public StreamProfile {
public:
    GyroStreamProfile( StreamProfile& profile );
    ~GyroStreamProfile();

    /**
     * @brief 获取满量程范围
     *
     * @return OBAccelFullScaleRange  返回量程范围值
     */
    OBGyroFullScaleRange fullScaleRange();

    /**
     * @brief 获取采样频率
     *
     * @return OBAccelFullScaleRange  返回采样频率
     */
    OBGyroSampleRate sampleRate();
};

template < typename T > bool StreamProfile::is() {
    switch ( this->type() ) {
    case OB_STREAM_VIDEO:
    case OB_STREAM_IR:
    case OB_STREAM_COLOR:
    case OB_STREAM_DEPTH:
        return typeid( T ) == typeid( VideoStreamProfile );
    case OB_STREAM_ACCEL:
        return typeid( T ) == typeid( AccelStreamProfile );
    case OB_STREAM_GYRO:
        return typeid( T ) == typeid( GyroStreamProfile );
    default:
        break;
    }
    return false;
}

class OB_EXTENSION_API StreamProfileList {
protected:
    std::unique_ptr< StreamProfileListImpl > impl_;

public:
    StreamProfileList( std::unique_ptr< StreamProfileListImpl > impl );
    ~StreamProfileList();

    /**
     * @brief 获取StreamProfile数量
     *
     * @return uint32_t 返回StreamProfile的数量
     */
    uint32_t count();

    /**
     * @brief 通过索引号获取StreamProfile
     *
     * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
     * @return std::shared_ptr<StreamProfile> 返回StreamProfile对象
     */
    const std::shared_ptr< StreamProfile > getProfile( uint32_t index );
};

}  // namespace ob
