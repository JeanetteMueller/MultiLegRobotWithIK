/**
 * AxisOffset.h
 *
 * Sideways offset of the knee joints (femur/tibia) along their rotation axis.
 *
 * Normally the coxa, femur and tibia servo form a straight line when viewed
 * from above (radially outward). Mechanically the knee joints may however be
 * mounted with a sideways offset (along their horizontal rotation axis, i.e.
 * tangential to the coxa direction). That offset is stored here per leg in mm
 * and accounted for in the IK.
 *
 * Default is always 0 -> classic straight leg arrangement.
 *
 * Author: Claude.ai & Jeanette Müller
 * Date: 2026
 */

#ifndef AxisOffset_H
#define AxisOffset_H

// Sideways offset of the two knee joints along their rotation axis (in mm).
// Positive = offset in the tangential direction (counter-clockwise around the
// body, same direction as a positive coxa angle).
struct AxisOffset
{
    float femur = 0.0f; // offset of the femur joint (at the coxa end)
    float tibia = 0.0f; // offset of the tibia joint (at the femur end)
};

#endif // AxisOffset_H
