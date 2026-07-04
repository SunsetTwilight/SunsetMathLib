#pragma once
/// @file Math.h
/// @brief Single include entry point for MathLib.
///
/// Include this one header to access all types and operations:
/// @code
/// #include "MathLib/Math.h"
/// @endcode
///
/// Dependency chain:
/// Math.h -> Quaternion.h -> MatrixN.h -> MatrixMul.h
///        -> VecMath.h -> VecTypes.h -> VecN.h
// ============================================================
//  Math.h  -- Single entry point
//
//  Include this single file to use all types and operations.
//
//  :
//    Vector types (VecTypes.h):
//      Float2 / Float3 / Float4
//      Double2 / Double3 / Double4
//      Int2 / Int3 / Int4
//      UInt2 / UInt3 / UInt4
//      Bool2 / Bool3 / Bool4
//      Vector2 / Vector3 / Vector4
//
//    Matrix types (MatrixN.h):
//      Matrix<T,R,C>
//      Matrix2x2 / Matrix3x3 / Matrix4x4
//      Matrix2x3 / Matrix2x4 / Matrix3x4 / Matrix4x3
//      DMatrix2x2 / DMatrix3x3 / DMatrix4x4
//      IMatrix2x2 / IMatrix3x3
//
//    Quaternion (Quaternion.h):
//      Quaternion
//
//  Provided operations (VecMath.h):
//    dot / lengthSq / length / distance
//    normalize / cross / reflect
//    lerp / perp / angle / project
//    select / clamp
//    *  Float2/3/4, Double2/3/4, Vector2/3/4 
//
//  Dependency graph:
//    Math.h
//      `-- Quaternion.h
//            `-- MatrixN.h
//                  |-- MatrixMul.h
//                  `-- VecMath.h
//                        `-- VecTypes.h
//                              `-- VecN.h
// ============================================================

#include "Quaternion.h"
#include "Transform.h"
#include "MathUtils.h"
#include "Geometry.h"
#include "Frustum.h"
#include "Triangle.h"
#include "OBB.h"
