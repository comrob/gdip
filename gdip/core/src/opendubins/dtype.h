/**
 * @file dtype.h
 * @author Petr Vana
 * @brief Types of Dubins maneuver
 */

#pragma once

#include <ostream>

namespace opendubins {

    enum struct DType {
        Unknown,

        RSR,
        LSL,
        RSL,
        LSR,
        RLR,
        LRL,

        // Dubins Interval Problem ----------------------------------------
        // CSC and CCC maneuvers used from Dubins maneuver
        DIP_S,
        DIP_Rp,
        DIP_Lp,
        DIP_RS,
        DIP_LS,
        DIP_SR,
        DIP_SL,
        DIP_RLp,
        DIP_LRp,
        DIP_RpL,
        DIP_LpR,
        DIP_LSL,
        DIP_RSR,
        DIP_LSR,
        DIP_RSL,
        DIP_LRL,
        DIP_RLR,

        // special cases - this should never happen
        DIP_LpRp,
        DIP_RpLp,

        // Generalized Dubins Interval Problem -------------------------
        GDIP_NO,
        GDIP_S,
        GDIP_R,
        GDIP_RS,
        GDIP_L,
        GDIP_LS,
        GDIP_SR,
        GDIP_SL,
        GDIP_RSR,
        GDIP_LSL,
        GDIP_RSL,
        GDIP_LSR,
        GDIP_LRL,
        GDIP_RLR,
        GDIP_Rp,
        GDIP_Lp,
        GDIP_LRp,
        GDIP_RLp,
        GDIP_LpR,
        GDIP_RpL,
    };

#define DTYPE_CASE(x) case DType::x: s << #x << "-" << (int)DType::x ; break;

    inline std::ostream& operator<< (std::ostream& s, DType type){
        switch (type){
            case DType::Unknown: s << "Unknown"; break;

            DTYPE_CASE(RSR)
            DTYPE_CASE(LSL)
            DTYPE_CASE(RSL)
            DTYPE_CASE(LSR)
            DTYPE_CASE(RLR)
            DTYPE_CASE(LRL)

            // DIP cases
            DTYPE_CASE(DIP_S)
            DTYPE_CASE(DIP_Rp)
            DTYPE_CASE(DIP_Lp)
            DTYPE_CASE(DIP_RS)
            DTYPE_CASE(DIP_LS)
            DTYPE_CASE(DIP_SR)
            DTYPE_CASE(DIP_SL)
            DTYPE_CASE(DIP_RLp)
            DTYPE_CASE(DIP_LRp)
            DTYPE_CASE(DIP_RpL)
            DTYPE_CASE(DIP_LpR)
            DTYPE_CASE(DIP_LSL)
            DTYPE_CASE(DIP_RSR)
            DTYPE_CASE(DIP_LSR)
            DTYPE_CASE(DIP_RSL)
            DTYPE_CASE(DIP_LRL)
            DTYPE_CASE(DIP_RLR)

            // GDIP cases
            DTYPE_CASE(GDIP_NO)
            DTYPE_CASE(GDIP_S)
            DTYPE_CASE(GDIP_R)
            DTYPE_CASE(GDIP_RS)
            DTYPE_CASE(GDIP_L)
            DTYPE_CASE(GDIP_LS)
            DTYPE_CASE(GDIP_SR)
            DTYPE_CASE(GDIP_SL)
            DTYPE_CASE(GDIP_RSR)
            DTYPE_CASE(GDIP_LSL)
            DTYPE_CASE(GDIP_RSL)
            DTYPE_CASE(GDIP_LSR)
            DTYPE_CASE(GDIP_LRL)
            DTYPE_CASE(GDIP_RLR)
            DTYPE_CASE(GDIP_Rp)
            DTYPE_CASE(GDIP_Lp)
            DTYPE_CASE(GDIP_LRp)
            DTYPE_CASE(GDIP_RLp)
            DTYPE_CASE(GDIP_LpR)
            DTYPE_CASE(GDIP_RpL)

            default: s << "Invalid("<<(int)type<<")"; break;
        }
        return s;
    }
}
