/***************************************************************************
 *   Copyright (c) 2023 Joel Meijering (EDG5000) <joel@meijering.email>    *
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

/*
Selection summary

Displays realtime measurements of the currently selected elements in bottom bar

When 1 sub-element is selected:
    Face: Display area
    Vertex, Wire: Display length
    Solid, Compound, Compound Solid: Display volume

When 2 sub-elements is selected:
    2 Faces:
        2 parallel faces: Display normal distance
        2 faces: display total area
    2 Edges
        2 parallel edges: Display x, y distance with Z being the edge vector (this can be rephrased more correctly)
        2 edges: display total length

        TODO
 */

#include "PreCompiled.h"
#ifndef _PreComp_
# include <QString>
# include <QTextStream>

/*
# include <assert.h>
# include <string>
# include <boost_bind_bind.hpp>
# include <QApplication>
# include <QStatusBar>
*/
#endif

#include <exception>
#include <sstream>

#include <Base/Console.h>
//#include <App/Application.h>
//#include <App/GeoFeature.h>
//#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/Selection.h>
#include <Gui/MainWindow.h>
#include <Mod/Part/App/PreCompiled.h>
#include <Mod/Part/App/PartFeature.h>
#include "SelectionSummary.h"

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepTools.hxx>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Ax1.hxx>
#include <gp_Lin.hxx>
#include <gp_Vec.hxx>

FC_LOG_LEVEL_INIT("SelectionSummary")

using namespace std;

namespace Gui{

constexpr double ANGULAR_TOLERANCE = 0;
constexpr double LINEAR_TOLERANCE = 0; // Maximum error passed to OCC when performing isNormal and isParallel checks

static gp_Ax1 edgeToAxis(TopoDS_Edge edge){
    auto vertex1 = TopExp::FirstVertex(edge);
    auto vertex2 = TopExp::LastVertex(edge);
    auto point1 = BRep_Tool::Pnt(vertex1);
    auto point2 = BRep_Tool::Pnt(vertex2);
    gp_Vec vec1(point1.XYZ());
    gp_Vec vec2(point2.XYZ());
    gp_Vec vecDir = vec1 - vec2;
    vecDir.Normalize();
    return gp_Ax1(BRep_Tool::Pnt(vertex1), vecDir);
}

static gp_Dir getNormal(TopoDS_Face face){
    Standard_Real umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    Handle(Geom_Surface) handle = BRep_Tool::Surface(face);
    GeomLProp_SLProps props(handle, umin, vmin, 1, 0.01);
    return props.Normal();
}

static float getFaceArea(TopoDS_Shape face){
    GProp_GProps gprops;
    BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
    return gprops.Mass();
}

// Should work with edges and wires
static float getLength(TopoDS_Shape wire){
    GProp_GProps gprops;
    BRepGProp::LinearProperties(wire, gprops); // Stores results in gprops
    return gprops.Mass();
}

static float getVolume(TopoDS_Shape shape){
    GProp_GProps gprops;
    BRepGProp::VolumeProperties(shape, gprops);
    return gprops.Mass();
}

void SelectionSummary::onSelectionChanged(const Gui::SelectionChanges& reason){
    try{
        QString text;
        QTextStream ts(&text);

        // Ignore events such as preselection (mouse hover). Measurement triggers for any mutation selection.
        if(reason.Type != SelectionChanges::AddSelection && reason.Type != SelectionChanges::RmvSelection && reason.Type != SelectionChanges::SetSelection && reason.Type != SelectionChanges::ClrSelection){
            return;
        }

        // Obtain list of selected objects
        //std::vector<App::DocumentObject*> selections = Selection().getObjectsOfType(App::GeoFeature::getClassTypeId());
        auto selections = Selection().getCompleteSelection();

        if(selections.size() == 0){
            return;
        }

        // For each selected object, obtain shape instance
        std::vector<TopoDS_Shape> shapeSelections;
        for(auto& selection: selections){
            TopoDS_Shape shape = Part::Feature::getShape(selection.pObject, selection.SubName, true);
            shapeSelections.push_back(shape);
        }

        // Generate different summary text based on amount of selections
        if(selections.size() == 1){
            // Display characteristics of selected element
            // Area of face
            // Circumference of face
            // Length of line
            // Radius and diameter of circle or circular face; length of circular face
            auto shape = shapeSelections.at(0);
            switch(shape.ShapeType()){
                case TopAbs_FACE: {
                    // Display face area
                    ts << "Face area: " << getFaceArea(shape);

                    // Check if face is cylindrial, if so display radius
                    auto face = TopoDS::Face(shape);
                    BRepAdaptor_Surface adaptor(face, Standard_False);
                     if(adaptor.GetType() == GeomAbs_Cylinder){
                        auto cylinder = adaptor.Cylinder();
                        auto radius = cylinder.Radius();
                        ts << " Cylindrical face radius: " << radius;
                    }

                    break;
                }
                case TopAbs_SHELL:
                    // Display shell volume
                    ts << "Shell volume: " << getVolume(shape);
                break;
                case TopAbs_SOLID:
                    // Display solid volume
                    ts << "Solid volume: " << getVolume(shape);
                break;
                case TopAbs_COMPOUND:
                    // Display compound volume
                    ts << "Compound volume: " << getVolume(shape);
                break;
                case TopAbs_COMPSOLID:
                    // Display compound solid volume
                    ts << "Compound solid volume: " << getVolume(shape);
                break;
                case TopAbs_WIRE:
                    // Display wire length
                    ts << "Wire length: " << getLength(shape);
                break;
                case TopAbs_EDGE:
                    // Display edge length
                    ts << "Edge length: " << getLength(shape);
                default:

                break;
            }
        }else if(selections.size() == 2){

            auto shapeA = shapeSelections.at(0);
            auto shapeB = shapeSelections.at(1);
            // Handle selection of two faces
            if(shapeA.ShapeType() == TopAbs_FACE && shapeA.ShapeType() == TopAbs_FACE){
                // Determine if both selections are cylindrical; if so, get axial distance
                auto faceA = TopoDS::Face(shapeA);
                auto faceB = TopoDS::Face(shapeB);
                BRepAdaptor_Surface surfAdaptorA(faceA, Standard_False);
                BRepAdaptor_Surface surfAdaptorB(faceB, Standard_False);

                double totalSurfaceArea = getFaceArea(shapeA) + getFaceArea(shapeB);
                ts << " Total surface area of faces: " << totalSurfaceArea;

                if(surfAdaptorA.GetType() == GeomAbs_Cylinder && surfAdaptorB.GetType() == GeomAbs_Cylinder){
                    // Obtain the axles of the two selected cylindrical faces
                    Handle(Geom_Surface) surfaceA = BRep_Tool::Surface(faceA);
                    Handle(Geom_CylindricalSurface) cylsurfA = Handle(Geom_CylindricalSurface)::DownCast(surfaceA);
                    auto axisA = cylsurfA->Cylinder().Axis();
                    Handle(Geom_Surface) surfaceB = BRep_Tool::Surface(faceB);
                    Handle(Geom_CylindricalSurface) cylsurfB = Handle(Geom_CylindricalSurface)::DownCast(surfaceB);
                    auto axisB = cylsurfB->Cylinder().Axis();

                    // If ciliders are parallel to each other but not coaxial, also display axial distance
                    if(axisA.IsParallel(axisB, ANGULAR_TOLERANCE) && !axisA.IsCoaxial(axisB, ANGULAR_TOLERANCE, LINEAR_TOLERANCE)){
                        // Two parallel cylindrical surfaces where selected, obtain the axial distance and display surface area
                        auto axialDistance = gp_Lin(axisA).Distance(gp_Lin(axisB));
                        ts << " Axial distance of cilindrical faces: " << axialDistance;
                    }
                }else if(surfAdaptorA.GetType() == GeomAbs_Plane && surfAdaptorB.GetType() == GeomAbs_Plane){
                    // Handle selection of two planar faces
                    // I faces are parallel to each other, display distance
                    if(getNormal(faceA).IsEqual(getNormal(faceB), ANGULAR_TOLERANCE) || getNormal(faceA).IsOpposite(getNormal(faceB), ANGULAR_TOLERANCE)){
                        // Faces are parallel, display distance and total surface area
                        BRepExtrema_DistShapeShape measure(shapeA, shapeB);
                        auto val = measure.Value();
                        ts << " Parallel face distance: " << val;
                    }
                }

            }else if(shapeA.ShapeType() == TopAbs_EDGE && shapeA.ShapeType() == TopAbs_EDGE){
                auto edgeA = TopoDS::Edge(shapeA);
                auto edgeB = TopoDS::Edge(shapeB);
                auto axisA = edgeToAxis(edgeA);
                auto axisB = edgeToAxis(edgeB);

                 ts << "Total length of edges: " << (getLength(shapeA) + getLength(shapeB));
                if(axisA.IsParallel(axisB, ANGULAR_TOLERANCE) && !axisA.IsCoaxial(axisB, ANGULAR_TOLERANCE, LINEAR_TOLERANCE)){
                    // Two parallel cylindrical surfaces where selected, obtain the axial distance and display surface area
                    auto axialDistance = gp_Lin(axisA).Distance(gp_Lin(axisB));
                    ts << " Axial distance: " << axialDistance;
                }
            }else if(shapeA.ShapeType() == TopAbs_VERTEX && shapeA.ShapeType() == TopAbs_VERTEX){
                // Handle selection of two vertices
                TopoDS_Vertex vertexA = TopoDS::Vertex(shapeA);
                TopoDS_Vertex vertexB = TopoDS::Vertex(shapeB);

                auto pointA = BRep_Tool::Pnt(vertexA);
                auto pointB = BRep_Tool::Pnt(vertexB);

                ts << "Distance between vertices: " << pointA.Distance(pointB) << " Delta X: " << pointA.X()-pointB.X() << " Delta Y: " << pointA.Y()-pointB.Y() << " Delta Z: " << pointA.Z()-pointB.Z();
            }else{
                // Handle heterogenous selection
                ts << "2 elements selected";
            }
        }else{
            // Check if all elements are of the same type
            auto type = shapeSelections.at(0).ShapeType();
            bool allTypesIdentical = true;
            for(auto& selection: shapeSelections){
                if(selection.ShapeType() != type){
                    allTypesIdentical = false;
                    break;
                }
            }
            if(allTypesIdentical){
                    switch(type){
                        case TopAbs_FACE: {
                            // All faces; display total area and selection count
                            float area = 0;
                            for(auto& shape: shapeSelections){
                                const TopoDS_Face &face = TopoDS::Face(shape);
                                area += getFaceArea(face);
                            }
                            ts << selections.size() << " faces. Total area: " << area;
                            break;
                        }
                        case TopAbs_EDGE: {
                            // All edges, display total edge count and selection count
                            float length = 0;
                            for(auto& shape: shapeSelections){
                                length += getLength(shape);
                            }
                            ts << selections.size() << " edges. Total length: " << length;
                            break;
                        }
                        case TopAbs_VERTEX: {
                            // All vertices, display count
                            ts << selections.size() << " vertices";
                            break;
                        }
                        case TopAbs_SOLID:
                        case TopAbs_COMPOUND:
                        case TopAbs_COMPSOLID: {
                            // Solids; display total volume
                            float volume = 0;
                            for(auto& shape: shapeSelections){
                                volume += getVolume(shape);
                            }
                            ts << selections.size() << " solids. Total volume: " << volume;
                            break;
                        }
                        default: {
                            ts << selections.size() << " selections";
                        }
                    }
            }else{
                // Different elements selected, display count
                ts << selections.size() << " selections";
            }

        }
        //FC_ERR((string) text.toUtf8());
        getMainWindow()->showMessage(text, 99999); // TODO this is hacky. Ideally the message would stay forever until something happens.
    }catch(Base::Exception& e){
        e.ReportException();
    }catch(exception& e){
        FC_ERR(e.what());
    }catch(Standard_Failure& e){
        stringstream ss;
        ss << e << endl;
        string err = ss.str();
        FC_ERR(err);
    }
}
}
