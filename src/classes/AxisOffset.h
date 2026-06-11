/**
 * AxisOffset.h
 *
 * Seitlicher Versatz der Knie-Gelenke (Femur/Tibia) entlang ihrer Drehachse.
 *
 * Normalerweise bilden Coxa-, Femur- und Tibia-Servo von oben betrachtet eine
 * gerade Linie (radial nach außen). Mechanisch kann es aber sein, dass die
 * Knie-Gelenke seitlich (entlang ihrer horizontalen Drehachse, also tangential
 * zur Coxa-Richtung) versetzt montiert sind. Dieser Versatz wird hier in mm
 * pro Bein hinterlegt und im IK berücksichtigt.
 *
 * Default ist immer 0 -> klassische gerade Beinanordnung.
 *
 * Autor: Claude.ai & Jeanette Müller
 * Datum: 2026
 */

#ifndef AxisOffset_H
#define AxisOffset_H

// Seitlicher Versatz der beiden Knie-Gelenke entlang ihrer Drehachse (in mm).
// Positiv = Versatz in tangentialer Richtung (gegen den Uhrzeigersinn um den
// Körper, gleiche Richtung wie ein positiver Coxa-Winkel).
struct AxisOffset
{
    float femur = 0.0f; // Versatz des Femur-Gelenks (am Coxa-Ende)
    float tibia = 0.0f; // Versatz des Tibia-Gelenks (am Femur-Ende)
};

#endif // AxisOffset_H
