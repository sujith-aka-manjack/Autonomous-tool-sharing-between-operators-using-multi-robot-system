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
                                                {1, CColor::BLUE},
                                                {2, CColor::ORANGE},
                                                {3, CColor::GREEN},
                                                {4, CColor::MAGENTA},
                                                {5, CColor::WHITE},
                                              };

#endif