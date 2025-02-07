/***************************************************************************
 *   Copyright (c) 2006 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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


#include "PreCompiled.h"

#ifndef _PreComp_
#endif

#include "Application.h"
#include "DocumentObjectGroup.h"
#include "DocumentObjectGroupPy.h"
#include "Document.h"
#include "FeaturePythonPyImp.h"
#include "GroupParams.h"

using namespace App;

PROPERTY_SOURCE_WITH_EXTENSIONS(App::DocumentObjectGroup, App::DocumentObject)

DocumentObjectGroup::DocumentObjectGroup(void): DocumentObject(), GroupExtension() {

    GroupExtension::initExtension(this);

    if(GroupParams::getExportChildren()) {
        ExportMode.setStatus(Property::Hidden,false);
        ExportMode.setValue(ExportByVisibility);
    }
}

DocumentObjectGroup::~DocumentObjectGroup() {

}

PyObject *DocumentObjectGroup::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectGroupPy(this),true);
    }
    return Py::new_reference_to(PythonObject);
}

// Python feature ---------------------------------------------------------

namespace App {

/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(App::DocumentObjectGroupPython, App::DocumentObjectGroup)

template<> void App::DocumentObjectGroupPython::setupObject() {
    ExportMode.setStatus(Property::Hidden,true);
    ExportMode.setValue(ExportDisabled);
}

template<> const char* App::DocumentObjectGroupPython::getViewProviderName(void) const {
    return "Gui::ViewProviderDocumentObjectGroupPython";
}
template<> PyObject* App::DocumentObjectGroupPython::getPyObject(void) {
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new FeaturePythonPyT<App::DocumentObjectGroupPy>(this),true);
    }
    return Py::new_reference_to(PythonObject);
}
/// @endcond

// explicit template instantiation
template class AppExport FeaturePythonT<App::DocumentObjectGroup>;
}
