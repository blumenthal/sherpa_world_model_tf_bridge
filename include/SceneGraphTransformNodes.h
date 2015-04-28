/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2015, KU Leuven
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef SCENEGRAPHTRANSFORMNODES_H_
#define SCENEGRAPHTRANSFORMNODES_H_

#include <brics_3d/worldModel/WorldModel.h>

const static unsigned int INVALID_ID = 0;

struct SceneGraphTransformNodes {
public:
	SceneGraphTransformNodes() : id(INVALID_ID){}
	brics_3d::rsg::Id id;
	std::string name;
	std::string tfParent;
};

#endif /* SCENEGRAPHTRANSFORMNODES_H_ */

/* EOF */
