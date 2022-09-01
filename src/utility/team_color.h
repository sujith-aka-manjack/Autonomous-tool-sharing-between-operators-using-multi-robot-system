/*
* AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
* 
* Defines the color to be used by each team.
*/

#ifndef TEAM_COLOR_H
#define TEAM_COLOR_H

/*
 * Include some necessary headers.
 */
/* Definition of the Color datatype. */
#include <argos3/core/utility/datatypes/color.h>
#include <unordered_map>

using namespace argos;

/* Team colors */
std::unordered_map<UInt8, CColor> teamColor = {
                                                {1, CColor::ORANGE},
                                                {2, CColor::BLUE},
                                                {3, CColor::BROWN},
                                                {4, CColor::GRAY30},
                                                {5, CColor::PURPLE},
                                                {6, CColor::RED},
                                                {7, CColor::GRAY50},
                                                {8, CColor::MAGENTA},
                                                {9, CColor::BLACK},
                                                {10, CColor::GREEN},
                                              };

#endif