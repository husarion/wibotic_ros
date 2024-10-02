/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/husarion/ros2_ws/src/wibotic/20200.WiBoticInfo.uavcan
 */

#ifndef WIBOTIC_WIBOTICINFO_HPP_INCLUDED
#define WIBOTIC_WIBOTICINFO_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# WiBotic periodic information.
#

#
# Primary parameters.
# Some fields can be set to NAN if their values are unknown.
#
float16 VMonBatt
float16 IBattery
float16 VRect
float16 VMonCharger
float16 TBoard
float16 TargetIBatt
float16 ICharger
float16 ISingleCharger2
float16 ISingleCharger3
******************************************************************************/

/********************* DSDL signature source definition ***********************
wibotic.WiBoticInfo
saturated float16 VMonBatt
saturated float16 IBattery
saturated float16 VRect
saturated float16 VMonCharger
saturated float16 TBoard
saturated float16 TargetIBatt
saturated float16 ICharger
saturated float16 ISingleCharger2
saturated float16 ISingleCharger3
******************************************************************************/

#undef VMonBatt
#undef IBattery
#undef VRect
#undef VMonCharger
#undef TBoard
#undef TargetIBatt
#undef ICharger
#undef ISingleCharger2
#undef ISingleCharger3

namespace wibotic
{

template <int _tmpl>
struct UAVCAN_EXPORT WiBoticInfo_
{
    typedef const WiBoticInfo_<_tmpl>& ParameterType;
    typedef WiBoticInfo_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > VMonBatt;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > IBattery;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > VRect;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > VMonCharger;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > TBoard;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > TargetIBatt;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > ICharger;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > ISingleCharger2;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > ISingleCharger3;
    };

    enum
    {
        MinBitLen
            = FieldTypes::VMonBatt::MinBitLen
            + FieldTypes::IBattery::MinBitLen
            + FieldTypes::VRect::MinBitLen
            + FieldTypes::VMonCharger::MinBitLen
            + FieldTypes::TBoard::MinBitLen
            + FieldTypes::TargetIBatt::MinBitLen
            + FieldTypes::ICharger::MinBitLen
            + FieldTypes::ISingleCharger2::MinBitLen
            + FieldTypes::ISingleCharger3::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::VMonBatt::MaxBitLen
            + FieldTypes::IBattery::MaxBitLen
            + FieldTypes::VRect::MaxBitLen
            + FieldTypes::VMonCharger::MaxBitLen
            + FieldTypes::TBoard::MaxBitLen
            + FieldTypes::TargetIBatt::MaxBitLen
            + FieldTypes::ICharger::MaxBitLen
            + FieldTypes::ISingleCharger2::MaxBitLen
            + FieldTypes::ISingleCharger3::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::VMonBatt >::Type VMonBatt;
    typename ::uavcan::StorageType< typename FieldTypes::IBattery >::Type IBattery;
    typename ::uavcan::StorageType< typename FieldTypes::VRect >::Type VRect;
    typename ::uavcan::StorageType< typename FieldTypes::VMonCharger >::Type VMonCharger;
    typename ::uavcan::StorageType< typename FieldTypes::TBoard >::Type TBoard;
    typename ::uavcan::StorageType< typename FieldTypes::TargetIBatt >::Type TargetIBatt;
    typename ::uavcan::StorageType< typename FieldTypes::ICharger >::Type ICharger;
    typename ::uavcan::StorageType< typename FieldTypes::ISingleCharger2 >::Type ISingleCharger2;
    typename ::uavcan::StorageType< typename FieldTypes::ISingleCharger3 >::Type ISingleCharger3;

    WiBoticInfo_()
        : VMonBatt()
        , IBattery()
        , VRect()
        , VMonCharger()
        , TBoard()
        , TargetIBatt()
        , ICharger()
        , ISingleCharger2()
        , ISingleCharger3()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<144 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 20200 };

    static const char* getDataTypeFullName()
    {
        return "wibotic.WiBoticInfo";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool WiBoticInfo_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        VMonBatt == rhs.VMonBatt &&
        IBattery == rhs.IBattery &&
        VRect == rhs.VRect &&
        VMonCharger == rhs.VMonCharger &&
        TBoard == rhs.TBoard &&
        TargetIBatt == rhs.TargetIBatt &&
        ICharger == rhs.ICharger &&
        ISingleCharger2 == rhs.ISingleCharger2 &&
        ISingleCharger3 == rhs.ISingleCharger3;
}

template <int _tmpl>
bool WiBoticInfo_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(VMonBatt, rhs.VMonBatt) &&
        ::uavcan::areClose(IBattery, rhs.IBattery) &&
        ::uavcan::areClose(VRect, rhs.VRect) &&
        ::uavcan::areClose(VMonCharger, rhs.VMonCharger) &&
        ::uavcan::areClose(TBoard, rhs.TBoard) &&
        ::uavcan::areClose(TargetIBatt, rhs.TargetIBatt) &&
        ::uavcan::areClose(ICharger, rhs.ICharger) &&
        ::uavcan::areClose(ISingleCharger2, rhs.ISingleCharger2) &&
        ::uavcan::areClose(ISingleCharger3, rhs.ISingleCharger3);
}

template <int _tmpl>
int WiBoticInfo_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::VMonBatt::encode(self.VMonBatt, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::IBattery::encode(self.IBattery, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::VRect::encode(self.VRect, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::VMonCharger::encode(self.VMonCharger, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::TBoard::encode(self.TBoard, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::TargetIBatt::encode(self.TargetIBatt, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ICharger::encode(self.ICharger, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ISingleCharger2::encode(self.ISingleCharger2, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ISingleCharger3::encode(self.ISingleCharger3, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int WiBoticInfo_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::VMonBatt::decode(self.VMonBatt, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::IBattery::decode(self.IBattery, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::VRect::decode(self.VRect, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::VMonCharger::decode(self.VMonCharger, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::TBoard::decode(self.TBoard, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::TargetIBatt::decode(self.TargetIBatt, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ICharger::decode(self.ICharger, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ISingleCharger2::decode(self.ISingleCharger2, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ISingleCharger3::decode(self.ISingleCharger3, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature WiBoticInfo_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xD7EF2EACD772E948ULL);

    FieldTypes::VMonBatt::extendDataTypeSignature(signature);
    FieldTypes::IBattery::extendDataTypeSignature(signature);
    FieldTypes::VRect::extendDataTypeSignature(signature);
    FieldTypes::VMonCharger::extendDataTypeSignature(signature);
    FieldTypes::TBoard::extendDataTypeSignature(signature);
    FieldTypes::TargetIBatt::extendDataTypeSignature(signature);
    FieldTypes::ICharger::extendDataTypeSignature(signature);
    FieldTypes::ISingleCharger2::extendDataTypeSignature(signature);
    FieldTypes::ISingleCharger3::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef WiBoticInfo_<0> WiBoticInfo;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::wibotic::WiBoticInfo > _uavcan_gdtr_registrator_WiBoticInfo;

}

} // Namespace wibotic

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::wibotic::WiBoticInfo >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::wibotic::WiBoticInfo::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::wibotic::WiBoticInfo >::stream(Stream& s, ::wibotic::WiBoticInfo::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "VMonBatt: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::VMonBatt >::stream(s, obj.VMonBatt, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "IBattery: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::IBattery >::stream(s, obj.IBattery, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "VRect: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::VRect >::stream(s, obj.VRect, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "VMonCharger: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::VMonCharger >::stream(s, obj.VMonCharger, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "TBoard: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::TBoard >::stream(s, obj.TBoard, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "TargetIBatt: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::TargetIBatt >::stream(s, obj.TargetIBatt, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ICharger: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::ICharger >::stream(s, obj.ICharger, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ISingleCharger2: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::ISingleCharger2 >::stream(s, obj.ISingleCharger2, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ISingleCharger3: ";
    YamlStreamer< ::wibotic::WiBoticInfo::FieldTypes::ISingleCharger3 >::stream(s, obj.ISingleCharger3, level + 1);
}

}

namespace wibotic
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::wibotic::WiBoticInfo::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::wibotic::WiBoticInfo >::stream(s, obj, 0);
    return s;
}

} // Namespace wibotic

#endif // WIBOTIC_WIBOTICINFO_HPP_INCLUDED