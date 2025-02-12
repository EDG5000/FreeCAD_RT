/***************************************************************************
 *   Copyright (c) 2010 Juergen Riegel <FreeCAD@juergen-riegel.net>        *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#ifndef PARTDESIGN_DressUp_H
#define PARTDESIGN_DressUp_H

#include <App/PropertyStandard.h>
#include "FeatureAddSub.h"

namespace PartDesign
{

class PartDesignExport DressUp : public PartDesign::FeatureAddSub
{
    PROPERTY_HEADER(PartDesign::DressUp);

public:
    DressUp();

    /**
     * Base feature and it's subelements to which dressup operation will be applied to.
     * Unlike Feature::BaseFeature it includes Sublinks and set not only inside a body.
     * But for consistency if BaseFeature is nonzero this links to the same body as it.
     */
    App::PropertyLinkSub Base;
    App::PropertyBool SupportTransform;
    Part::PropertyPartShape   DressUpShape;

    short mustExecute() const;
    /// updates the Placement property from the Placement of the BaseFeature
    void positionByBaseFeature(void);
    /**
     * Returns the BaseFeature property's object if it's set otherwise returns Base's
     * feature property object otherwise feature property's object (if any)
     * @param silent if couldn't determine the base feature and silent == true,
     *               silently return a nullptr, otherwise throw Base::Exception.
     *               Default is false.
     */
    virtual Part::Feature* getBaseObject(bool silent=false) const;
    /// extracts all edges from the subshapes (including face edges) and furthermore adds
    /// all C0 continuous edges to the vector
    std::vector<TopoShape> getContinuousEdges(const TopoShape &shape);

    std::vector<TopoShape> getFaces(const TopoShape &shape);

    virtual void getAddSubShape(std::vector<std::pair<Part::TopoShape, Type> > &shapes);

    static const std::string &addsubElementPrefix();

    virtual void setPauseRecompute(bool) {}

protected:
    virtual void onChanged(const App::Property* prop);
};

} //namespace PartDesign


#endif // PARTDESIGN_DressUp_H
