// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"

#include <array>

namespace carla {
namespace geom {

  class SimpleTrajectoryPoint {
  public:

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float v = 0.0f;

    // =========================================================================
    // -- Constructors ---------------------------------------------------------
    // =========================================================================

    SimpleTrajectoryPoint() : x(0.0f), y(0.0f), z(0.0f), v(0.0f) {}

    SimpleTrajectoryPoint(const SimpleTrajectoryPoint &pt) {
      x = pt.x;
      y = pt.y;
      z = pt.z;
      v = pt.v;
    };

    SimpleTrajectoryPoint(float tx, float ty, float tz, float tv) : x(tx), y(ty), z(tz), v(tv) {}

    // =========================================================================
    /// @todo The following is copy-pasted from MSGPACK_DEFINE_ARRAY.
    /// This is a workaround for an issue in msgpack library. The
    /// MSGPACK_DEFINE_ARRAY macro is shadowing our `z` variable.
    /// https://github.com/msgpack/msgpack-c/issues/709
    // =========================================================================
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_pack(pk);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o)
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_unpack(o);
    }
    template <typename MSGPACK_OBJECT>
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& sneaky_variable_that_shadows_z) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_object(o, sneaky_variable_that_shadows_z);
    }
    // =========================================================================
  };

} // namespace geom
} // namespace carla
