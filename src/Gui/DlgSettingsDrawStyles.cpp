/****************************************************************************
 *   Copyright (c) 2020 Zheng Lei (realthunder) <realthunder.dev@gmail.com> *
 *                                                                          *
 *   This file is part of the FreeCAD CAx development system.               *
 *                                                                          *
 *   This library is free software; you can redistribute it and/or          *
 *   modify it under the terms of the GNU Library General Public            *
 *   License as published by the Free Software Foundation; either           *
 *   version 2 of the License, or (at your option) any later version.       *
 *                                                                          *
 *   This library  is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU Library General Public License for more details.                   *
 *                                                                          *
 *   You should have received a copy of the GNU Library General Public      *
 *   License along with this library; see the file COPYING.LIB. If not,     *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,          *
 *   Suite 330, Boston, MA  02111-1307, USA                                 *
 *                                                                          *
 ****************************************************************************/

#include "PreCompiled.h"

#include <App/Material.h>

/*[[[cog
import DlgSettingsDrawStyles
DlgSettingsDrawStyles.define()
]]]*/

// Auto generated code (Tools/params_utils.py:467)
#ifndef _PreComp_
#   include <QApplication>
#   include <QLabel>
#   include <QGroupBox>
#   include <QGridLayout>
#   include <QVBoxLayout>
#   include <QHBoxLayout>
#endif
#include <Gui/ViewParams.h>
// Auto generated code (Tools/params_utils.py:485)
#include "Gui/DlgSettingsDrawStyles.h"
using namespace Gui::Dialog;
/* TRANSLATOR Gui::Dialog::DlgSettingsDrawStyles */

// Auto generated code (Tools/params_utils.py:492)
DlgSettingsDrawStyles::DlgSettingsDrawStyles(QWidget* parent)
    : PreferencePage( parent )
{

    auto layout = new QVBoxLayout(this);


    // Auto generated code (Tools/params_utils.py:342)
    groupGeneral = new QGroupBox(this);
    layout->addWidget(groupGeneral);
    auto layoutHorizGeneral = new QHBoxLayout(groupGeneral);
    auto layoutGeneral = new QGridLayout();
    layoutHorizGeneral->addLayout(layoutGeneral);
    layoutHorizGeneral->addStretch();

    // Auto generated code (Tools/params_utils.py:353)
    labelDefaultDrawStyle = new QLabel(this);
    layoutGeneral->addWidget(labelDefaultDrawStyle, 0, 0);
    DefaultDrawStyle = new Gui::PrefComboBox(this);
    layoutGeneral->addWidget(DefaultDrawStyle, 0, 1);
    DefaultDrawStyle->setEntryName("DefaultDrawStyle");
    DefaultDrawStyle->setParamGrpPath("View");
    for (int i=0; i<9; ++i) // Auto generated code (Tools/params_utils.py:878)
        DefaultDrawStyle->addItem(QString());
    DefaultDrawStyle->setCurrentIndex(Gui::ViewParams::defaultDefaultDrawStyle());

    // Auto generated code (Tools/params_utils.py:353)
    ForceSolidSingleSideLighting = new Gui::PrefCheckBox(this);
    layoutGeneral->addWidget(ForceSolidSingleSideLighting, 1, 0);
    ForceSolidSingleSideLighting->setChecked(Gui::ViewParams::defaultForceSolidSingleSideLighting());
    ForceSolidSingleSideLighting->setEntryName("ForceSolidSingleSideLighting");
    ForceSolidSingleSideLighting->setParamGrpPath("View");


    // Auto generated code (Tools/params_utils.py:342)
    groupSelection = new QGroupBox(this);
    layout->addWidget(groupSelection);
    auto layoutHorizSelection = new QHBoxLayout(groupSelection);
    auto layoutSelection = new QGridLayout();
    layoutHorizSelection->addLayout(layoutSelection);
    layoutHorizSelection->addStretch();

    // Auto generated code (Tools/params_utils.py:353)
    labelTransparencyOnTop = new QLabel(this);
    layoutSelection->addWidget(labelTransparencyOnTop, 0, 0);
    TransparencyOnTop = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(TransparencyOnTop, 0, 1);
    TransparencyOnTop->setValue(Gui::ViewParams::defaultTransparencyOnTop());
    TransparencyOnTop->setEntryName("TransparencyOnTop");
    TransparencyOnTop->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionLineThicken = new QLabel(this);
    layoutSelection->addWidget(labelSelectionLineThicken, 1, 0);
    SelectionLineThicken = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(SelectionLineThicken, 1, 1);
    SelectionLineThicken->setValue(Gui::ViewParams::defaultSelectionLineThicken());
    SelectionLineThicken->setEntryName("SelectionLineThicken");
    SelectionLineThicken->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionLineMaxWidth = new QLabel(this);
    layoutSelection->addWidget(labelSelectionLineMaxWidth, 2, 0);
    SelectionLineMaxWidth = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(SelectionLineMaxWidth, 2, 1);
    SelectionLineMaxWidth->setValue(Gui::ViewParams::defaultSelectionLineMaxWidth());
    SelectionLineMaxWidth->setEntryName("SelectionLineMaxWidth");
    SelectionLineMaxWidth->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionPointScale = new QLabel(this);
    layoutSelection->addWidget(labelSelectionPointScale, 3, 0);
    SelectionPointScale = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(SelectionPointScale, 3, 1);
    SelectionPointScale->setValue(Gui::ViewParams::defaultSelectionPointScale());
    SelectionPointScale->setEntryName("SelectionPointScale");
    SelectionPointScale->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionPointMaxSize = new QLabel(this);
    layoutSelection->addWidget(labelSelectionPointMaxSize, 4, 0);
    SelectionPointMaxSize = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(SelectionPointMaxSize, 4, 1);
    SelectionPointMaxSize->setValue(Gui::ViewParams::defaultSelectionPointMaxSize());
    SelectionPointMaxSize->setEntryName("SelectionPointMaxSize");
    SelectionPointMaxSize->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionLinePattern = new QLabel(this);
    layoutSelection->addWidget(labelSelectionLinePattern, 5, 0);
    SelectionLinePattern = new Gui::PrefLinePattern(this);
    layoutSelection->addWidget(SelectionLinePattern, 5, 1);
    SelectionLinePattern->setEntryName("SelectionLinePattern");
    SelectionLinePattern->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:913)
    for (int i=1; i<SelectionLinePattern->count(); ++i) {
        if (SelectionLinePattern->itemData(i).toInt() == 0)
            SelectionLinePattern->setCurrentIndex(i);
    }

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionLinePatternScale = new QLabel(this);
    layoutSelection->addWidget(labelSelectionLinePatternScale, 6, 0);
    SelectionLinePatternScale = new Gui::PrefSpinBox(this);
    layoutSelection->addWidget(SelectionLinePatternScale, 6, 1);
    SelectionLinePatternScale->setValue(Gui::ViewParams::defaultSelectionLinePatternScale());
    SelectionLinePatternScale->setEntryName("SelectionLinePatternScale");
    SelectionLinePatternScale->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelSelectionHiddenLineWidth = new QLabel(this);
    layoutSelection->addWidget(labelSelectionHiddenLineWidth, 7, 0);
    SelectionHiddenLineWidth = new Gui::PrefDoubleSpinBox(this);
    layoutSelection->addWidget(SelectionHiddenLineWidth, 7, 1);
    SelectionHiddenLineWidth->setValue(Gui::ViewParams::defaultSelectionHiddenLineWidth());
    SelectionHiddenLineWidth->setEntryName("SelectionHiddenLineWidth");
    SelectionHiddenLineWidth->setParamGrpPath("View");


    // Auto generated code (Tools/params_utils.py:342)
    groupHiddenLines = new QGroupBox(this);
    layout->addWidget(groupHiddenLines);
    auto layoutHorizHiddenLines = new QHBoxLayout(groupHiddenLines);
    auto layoutHiddenLines = new QGridLayout();
    layoutHorizHiddenLines->addLayout(layoutHiddenLines);
    layoutHorizHiddenLines->addStretch();

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineFaceColor = new Gui::PrefColorButton(this);
    layoutHiddenLines->addWidget(HiddenLineFaceColor, 0, 1);
    HiddenLineFaceColor->setPackedColor(Gui::ViewParams::defaultHiddenLineFaceColor());
    HiddenLineFaceColor->setEntryName("HiddenLineFaceColor");
    HiddenLineFaceColor->setParamGrpPath("View");
    HiddenLineOverrideFaceColor = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineOverrideFaceColor, 0, 0);
    HiddenLineOverrideFaceColor->setChecked(Gui::ViewParams::defaultHiddenLineOverrideFaceColor());
    HiddenLineOverrideFaceColor->setEntryName("HiddenLineOverrideFaceColor");
    HiddenLineOverrideFaceColor->setParamGrpPath("View");
    HiddenLineFaceColor->setEnabled(HiddenLineOverrideFaceColor->isChecked());
    connect(HiddenLineOverrideFaceColor, SIGNAL(toggled(bool)), HiddenLineFaceColor, SLOT(setEnabled(bool)));
    HiddenLineFaceColor->setAllowTransparency(true);

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineColor = new Gui::PrefColorButton(this);
    layoutHiddenLines->addWidget(HiddenLineColor, 1, 1);
    HiddenLineColor->setPackedColor(Gui::ViewParams::defaultHiddenLineColor());
    HiddenLineColor->setEntryName("HiddenLineColor");
    HiddenLineColor->setParamGrpPath("View");
    HiddenLineOverrideColor = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineOverrideColor, 1, 0);
    HiddenLineOverrideColor->setChecked(Gui::ViewParams::defaultHiddenLineOverrideColor());
    HiddenLineOverrideColor->setEntryName("HiddenLineOverrideColor");
    HiddenLineOverrideColor->setParamGrpPath("View");
    HiddenLineColor->setEnabled(HiddenLineOverrideColor->isChecked());
    connect(HiddenLineOverrideColor, SIGNAL(toggled(bool)), HiddenLineColor, SLOT(setEnabled(bool)));
    HiddenLineColor->setAllowTransparency(true);

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineBackground = new Gui::PrefColorButton(this);
    layoutHiddenLines->addWidget(HiddenLineBackground, 2, 1);
    HiddenLineBackground->setPackedColor(Gui::ViewParams::defaultHiddenLineBackground());
    HiddenLineBackground->setEntryName("HiddenLineBackground");
    HiddenLineBackground->setParamGrpPath("View");
    HiddenLineOverrideBackground = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineOverrideBackground, 2, 0);
    HiddenLineOverrideBackground->setChecked(Gui::ViewParams::defaultHiddenLineOverrideBackground());
    HiddenLineOverrideBackground->setEntryName("HiddenLineOverrideBackground");
    HiddenLineOverrideBackground->setParamGrpPath("View");
    HiddenLineBackground->setEnabled(HiddenLineOverrideBackground->isChecked());
    connect(HiddenLineOverrideBackground, SIGNAL(toggled(bool)), HiddenLineBackground, SLOT(setEnabled(bool)));
    HiddenLineBackground->setAllowTransparency(true);

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineShaded = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineShaded, 3, 0);
    HiddenLineShaded->setChecked(Gui::ViewParams::defaultHiddenLineShaded());
    HiddenLineShaded->setEntryName("HiddenLineShaded");
    HiddenLineShaded->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineShowOutline = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineShowOutline, 4, 0);
    HiddenLineShowOutline->setChecked(Gui::ViewParams::defaultHiddenLineShowOutline());
    HiddenLineShowOutline->setEntryName("HiddenLineShowOutline");
    HiddenLineShowOutline->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLinePerFaceOutline = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLinePerFaceOutline, 5, 0);
    HiddenLinePerFaceOutline->setChecked(Gui::ViewParams::defaultHiddenLinePerFaceOutline());
    HiddenLinePerFaceOutline->setEntryName("HiddenLinePerFaceOutline");
    HiddenLinePerFaceOutline->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineSceneOutline = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineSceneOutline, 6, 0);
    HiddenLineSceneOutline->setChecked(Gui::ViewParams::defaultHiddenLineSceneOutline());
    HiddenLineSceneOutline->setEntryName("HiddenLineSceneOutline");
    HiddenLineSceneOutline->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelHiddenLineOutlineWidth = new QLabel(this);
    layoutHiddenLines->addWidget(labelHiddenLineOutlineWidth, 7, 0);
    HiddenLineOutlineWidth = new Gui::PrefDoubleSpinBox(this);
    layoutHiddenLines->addWidget(HiddenLineOutlineWidth, 7, 1);
    HiddenLineOutlineWidth->setValue(Gui::ViewParams::defaultHiddenLineOutlineWidth());
    HiddenLineOutlineWidth->setEntryName("HiddenLineOutlineWidth");
    HiddenLineOutlineWidth->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    HiddenLineOutlineWidth->setMinimum(0.0);
    HiddenLineOutlineWidth->setMaximum(100.0);
    HiddenLineOutlineWidth->setSingleStep(0.5);

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineHideFace = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineHideFace, 8, 0);
    HiddenLineHideFace->setChecked(Gui::ViewParams::defaultHiddenLineHideFace());
    HiddenLineHideFace->setEntryName("HiddenLineHideFace");
    HiddenLineHideFace->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineHideSeam = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineHideSeam, 9, 0);
    HiddenLineHideSeam->setChecked(Gui::ViewParams::defaultHiddenLineHideSeam());
    HiddenLineHideSeam->setEntryName("HiddenLineHideSeam");
    HiddenLineHideSeam->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineHideVertex = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineHideVertex, 10, 0);
    HiddenLineHideVertex->setChecked(Gui::ViewParams::defaultHiddenLineHideVertex());
    HiddenLineHideVertex->setEntryName("HiddenLineHideVertex");
    HiddenLineHideVertex->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    HiddenLineTransparency = new Gui::PrefDoubleSpinBox(this);
    layoutHiddenLines->addWidget(HiddenLineTransparency, 11, 1);
    HiddenLineTransparency->setValue(Gui::ViewParams::defaultHiddenLineTransparency());
    HiddenLineTransparency->setEntryName("HiddenLineTransparency");
    HiddenLineTransparency->setParamGrpPath("View");
    HiddenLineOverrideTransparency = new Gui::PrefCheckBox(this);
    layoutHiddenLines->addWidget(HiddenLineOverrideTransparency, 11, 0);
    HiddenLineOverrideTransparency->setChecked(Gui::ViewParams::defaultHiddenLineOverrideTransparency());
    HiddenLineOverrideTransparency->setEntryName("HiddenLineOverrideTransparency");
    HiddenLineOverrideTransparency->setParamGrpPath("View");
    HiddenLineTransparency->setEnabled(HiddenLineOverrideTransparency->isChecked());
    connect(HiddenLineOverrideTransparency, SIGNAL(toggled(bool)), HiddenLineTransparency, SLOT(setEnabled(bool)));

    // Auto generated code (Tools/params_utils.py:353)
    labelHiddenLineWidth = new QLabel(this);
    layoutHiddenLines->addWidget(labelHiddenLineWidth, 12, 0);
    HiddenLineWidth = new Gui::PrefDoubleSpinBox(this);
    layoutHiddenLines->addWidget(HiddenLineWidth, 12, 1);
    HiddenLineWidth->setValue(Gui::ViewParams::defaultHiddenLineWidth());
    HiddenLineWidth->setEntryName("HiddenLineWidth");
    HiddenLineWidth->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelHiddenLinePointSize = new QLabel(this);
    layoutHiddenLines->addWidget(labelHiddenLinePointSize, 13, 0);
    HiddenLinePointSize = new Gui::PrefDoubleSpinBox(this);
    layoutHiddenLines->addWidget(HiddenLinePointSize, 13, 1);
    HiddenLinePointSize->setValue(Gui::ViewParams::defaultHiddenLinePointSize());
    HiddenLinePointSize->setEntryName("HiddenLinePointSize");
    HiddenLinePointSize->setParamGrpPath("View");


    // Auto generated code (Tools/params_utils.py:342)
    groupShadow = new QGroupBox(this);
    layout->addWidget(groupShadow);
    auto layoutHorizShadow = new QHBoxLayout(groupShadow);
    auto layoutShadow = new QGridLayout();
    layoutHorizShadow->addLayout(layoutShadow);
    layoutHorizShadow->addStretch();

    // Auto generated code (Tools/params_utils.py:353)
    ShadowSpotLight = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowSpotLight, 0, 0);
    ShadowSpotLight->setChecked(Gui::ViewParams::defaultShadowSpotLight());
    ShadowSpotLight->setEntryName("ShadowSpotLight");
    ShadowSpotLight->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowLightColor = new QLabel(this);
    layoutShadow->addWidget(labelShadowLightColor, 1, 0);
    ShadowLightColor = new Gui::PrefColorButton(this);
    layoutShadow->addWidget(ShadowLightColor, 1, 1);
    ShadowLightColor->setPackedColor(Gui::ViewParams::defaultShadowLightColor());
    ShadowLightColor->setEntryName("ShadowLightColor");
    ShadowLightColor->setParamGrpPath("View");
    ShadowLightColor->setAllowTransparency(true);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowLightIntensity = new QLabel(this);
    layoutShadow->addWidget(labelShadowLightIntensity, 2, 0);
    ShadowLightIntensity = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowLightIntensity, 2, 1);
    ShadowLightIntensity->setValue(Gui::ViewParams::defaultShadowLightIntensity());
    ShadowLightIntensity->setEntryName("ShadowLightIntensity");
    ShadowLightIntensity->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    ShadowShowGround = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowShowGround, 3, 0);
    ShadowShowGround->setChecked(Gui::ViewParams::defaultShadowShowGround());
    ShadowShowGround->setEntryName("ShadowShowGround");
    ShadowShowGround->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    ShadowGroundBackFaceCull = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowGroundBackFaceCull, 4, 0);
    ShadowGroundBackFaceCull->setChecked(Gui::ViewParams::defaultShadowGroundBackFaceCull());
    ShadowGroundBackFaceCull->setEntryName("ShadowGroundBackFaceCull");
    ShadowGroundBackFaceCull->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundColor = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundColor, 5, 0);
    ShadowGroundColor = new Gui::PrefColorButton(this);
    layoutShadow->addWidget(ShadowGroundColor, 5, 1);
    ShadowGroundColor->setPackedColor(Gui::ViewParams::defaultShadowGroundColor());
    ShadowGroundColor->setEntryName("ShadowGroundColor");
    ShadowGroundColor->setParamGrpPath("View");
    ShadowGroundColor->setAllowTransparency(true);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundScale = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundScale, 6, 0);
    ShadowGroundScale = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowGroundScale, 6, 1);
    ShadowGroundScale->setValue(Gui::ViewParams::defaultShadowGroundScale());
    ShadowGroundScale->setEntryName("ShadowGroundScale");
    ShadowGroundScale->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowGroundScale->setMinimum(0.0);
    ShadowGroundScale->setMaximum(10000000.0);
    ShadowGroundScale->setSingleStep(0.5);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundTransparency = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundTransparency, 7, 0);
    ShadowGroundTransparency = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowGroundTransparency, 7, 1);
    ShadowGroundTransparency->setValue(Gui::ViewParams::defaultShadowGroundTransparency());
    ShadowGroundTransparency->setEntryName("ShadowGroundTransparency");
    ShadowGroundTransparency->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowGroundTransparency->setMinimum(0.0);
    ShadowGroundTransparency->setMaximum(1.0);
    ShadowGroundTransparency->setSingleStep(0.1);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundTexture = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundTexture, 8, 0);
    ShadowGroundTexture = new Gui::PrefFileChooser(this);
    layoutShadow->addWidget(ShadowGroundTexture, 8, 1);
    ShadowGroundTexture->setFileNameStd(Gui::ViewParams::defaultShadowGroundTexture());
    ShadowGroundTexture->setEntryName("ShadowGroundTexture");
    ShadowGroundTexture->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundTextureSize = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundTextureSize, 9, 0);
    ShadowGroundTextureSize = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowGroundTextureSize, 9, 1);
    ShadowGroundTextureSize->setValue(Gui::ViewParams::defaultShadowGroundTextureSize());
    ShadowGroundTextureSize->setEntryName("ShadowGroundTextureSize");
    ShadowGroundTextureSize->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowGroundTextureSize->setMinimum(0.0);
    ShadowGroundTextureSize->setMaximum(10000000.0);
    ShadowGroundTextureSize->setSingleStep(10.0);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowGroundBumpMap = new QLabel(this);
    layoutShadow->addWidget(labelShadowGroundBumpMap, 10, 0);
    ShadowGroundBumpMap = new Gui::PrefFileChooser(this);
    layoutShadow->addWidget(ShadowGroundBumpMap, 10, 1);
    ShadowGroundBumpMap->setFileNameStd(Gui::ViewParams::defaultShadowGroundBumpMap());
    ShadowGroundBumpMap->setEntryName("ShadowGroundBumpMap");
    ShadowGroundBumpMap->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    ShadowGroundShading = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowGroundShading, 11, 0);
    ShadowGroundShading->setChecked(Gui::ViewParams::defaultShadowGroundShading());
    ShadowGroundShading->setEntryName("ShadowGroundShading");
    ShadowGroundShading->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    ShadowUpdateGround = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowUpdateGround, 12, 0);
    ShadowUpdateGround->setChecked(Gui::ViewParams::defaultShadowUpdateGround());
    ShadowUpdateGround->setEntryName("ShadowUpdateGround");
    ShadowUpdateGround->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowDisplayMode = new QLabel(this);
    layoutShadow->addWidget(labelShadowDisplayMode, 13, 0);
    ShadowDisplayMode = new Gui::PrefComboBox(this);
    layoutShadow->addWidget(ShadowDisplayMode, 13, 1);
    ShadowDisplayMode->setEntryName("ShadowDisplayMode");
    ShadowDisplayMode->setParamGrpPath("View");
    for (int i=0; i<3; ++i) // Auto generated code (Tools/params_utils.py:878)
        ShadowDisplayMode->addItem(QString());
    ShadowDisplayMode->setCurrentIndex(Gui::ViewParams::defaultShadowDisplayMode());

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowPrecision = new QLabel(this);
    layoutShadow->addWidget(labelShadowPrecision, 14, 0);
    ShadowPrecision = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowPrecision, 14, 1);
    ShadowPrecision->setValue(Gui::ViewParams::defaultShadowPrecision());
    ShadowPrecision->setEntryName("ShadowPrecision");
    ShadowPrecision->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowPrecision->setMinimum(0.0);
    ShadowPrecision->setMaximum(1.0);
    ShadowPrecision->setSingleStep(0.1);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowSmoothBorder = new QLabel(this);
    layoutShadow->addWidget(labelShadowSmoothBorder, 15, 0);
    ShadowSmoothBorder = new Gui::PrefSpinBox(this);
    layoutShadow->addWidget(ShadowSmoothBorder, 15, 1);
    ShadowSmoothBorder->setValue(Gui::ViewParams::defaultShadowSmoothBorder());
    ShadowSmoothBorder->setEntryName("ShadowSmoothBorder");
    ShadowSmoothBorder->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowSpreadSize = new QLabel(this);
    layoutShadow->addWidget(labelShadowSpreadSize, 16, 0);
    ShadowSpreadSize = new Gui::PrefSpinBox(this);
    layoutShadow->addWidget(ShadowSpreadSize, 16, 1);
    ShadowSpreadSize->setValue(Gui::ViewParams::defaultShadowSpreadSize());
    ShadowSpreadSize->setEntryName("ShadowSpreadSize");
    ShadowSpreadSize->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowSpreadSize->setMinimum(0);
    ShadowSpreadSize->setMaximum(10000000.0);
    ShadowSpreadSize->setSingleStep(500);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowSpreadSampleSize = new QLabel(this);
    layoutShadow->addWidget(labelShadowSpreadSampleSize, 17, 0);
    ShadowSpreadSampleSize = new Gui::PrefSpinBox(this);
    layoutShadow->addWidget(ShadowSpreadSampleSize, 17, 1);
    ShadowSpreadSampleSize->setValue(Gui::ViewParams::defaultShadowSpreadSampleSize());
    ShadowSpreadSampleSize->setEntryName("ShadowSpreadSampleSize");
    ShadowSpreadSampleSize->setParamGrpPath("View");

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowEpsilon = new QLabel(this);
    layoutShadow->addWidget(labelShadowEpsilon, 18, 0);
    ShadowEpsilon = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowEpsilon, 18, 1);
    ShadowEpsilon->setValue(Gui::ViewParams::defaultShadowEpsilon());
    ShadowEpsilon->setEntryName("ShadowEpsilon");
    ShadowEpsilon->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowEpsilon->setMinimum(0.0);
    ShadowEpsilon->setMaximum(1.0);
    ShadowEpsilon->setSingleStep(1e-05);
    ShadowEpsilon->setDecimals(10);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowThreshold = new QLabel(this);
    layoutShadow->addWidget(labelShadowThreshold, 19, 0);
    ShadowThreshold = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowThreshold, 19, 1);
    ShadowThreshold->setValue(Gui::ViewParams::defaultShadowThreshold());
    ShadowThreshold->setEntryName("ShadowThreshold");
    ShadowThreshold->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowThreshold->setMinimum(0.0);
    ShadowThreshold->setMaximum(1.0);
    ShadowThreshold->setSingleStep(0.1);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowBoundBoxScale = new QLabel(this);
    layoutShadow->addWidget(labelShadowBoundBoxScale, 20, 0);
    ShadowBoundBoxScale = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowBoundBoxScale, 20, 1);
    ShadowBoundBoxScale->setValue(Gui::ViewParams::defaultShadowBoundBoxScale());
    ShadowBoundBoxScale->setEntryName("ShadowBoundBoxScale");
    ShadowBoundBoxScale->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowBoundBoxScale->setMinimum(0.0);
    ShadowBoundBoxScale->setMaximum(10000000.0);
    ShadowBoundBoxScale->setSingleStep(0.5);

    // Auto generated code (Tools/params_utils.py:353)
    labelShadowMaxDistance = new QLabel(this);
    layoutShadow->addWidget(labelShadowMaxDistance, 21, 0);
    ShadowMaxDistance = new Gui::PrefDoubleSpinBox(this);
    layoutShadow->addWidget(ShadowMaxDistance, 21, 1);
    ShadowMaxDistance->setValue(Gui::ViewParams::defaultShadowMaxDistance());
    ShadowMaxDistance->setEntryName("ShadowMaxDistance");
    ShadowMaxDistance->setParamGrpPath("View");
    // Auto generated code (Tools/params_utils.py:948)
    ShadowMaxDistance->setMinimum(0.0);
    ShadowMaxDistance->setMaximum(10000000.0);
    ShadowMaxDistance->setSingleStep(0.5);

    // Auto generated code (Tools/params_utils.py:353)
    ShadowTransparentShadow = new Gui::PrefCheckBox(this);
    layoutShadow->addWidget(ShadowTransparentShadow, 22, 0);
    ShadowTransparentShadow->setChecked(Gui::ViewParams::defaultShadowTransparentShadow());
    ShadowTransparentShadow->setEntryName("ShadowTransparentShadow");
    ShadowTransparentShadow->setParamGrpPath("View");
    layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Expanding));
    retranslateUi();
    // Auto generated code (Tools/params_utils.py:499)
    
}

// Auto generated code (Tools/params_utils.py:504)
DlgSettingsDrawStyles::~DlgSettingsDrawStyles()
{
}

// Auto generated code (Tools/params_utils.py:510)
void DlgSettingsDrawStyles::saveSettings()
{
    // Auto generated code (Tools/params_utils.py:373)
    DefaultDrawStyle->onSave();
    ForceSolidSingleSideLighting->onSave();
    TransparencyOnTop->onSave();
    SelectionLineThicken->onSave();
    SelectionLineMaxWidth->onSave();
    SelectionPointScale->onSave();
    SelectionPointMaxSize->onSave();
    SelectionLinePattern->onSave();
    SelectionLinePatternScale->onSave();
    SelectionHiddenLineWidth->onSave();
    HiddenLineFaceColor->onSave();
    HiddenLineOverrideFaceColor->onSave();
    HiddenLineColor->onSave();
    HiddenLineOverrideColor->onSave();
    HiddenLineBackground->onSave();
    HiddenLineOverrideBackground->onSave();
    HiddenLineShaded->onSave();
    HiddenLineShowOutline->onSave();
    HiddenLinePerFaceOutline->onSave();
    HiddenLineSceneOutline->onSave();
    HiddenLineOutlineWidth->onSave();
    HiddenLineHideFace->onSave();
    HiddenLineHideSeam->onSave();
    HiddenLineHideVertex->onSave();
    HiddenLineTransparency->onSave();
    HiddenLineOverrideTransparency->onSave();
    HiddenLineWidth->onSave();
    HiddenLinePointSize->onSave();
    ShadowSpotLight->onSave();
    ShadowLightColor->onSave();
    ShadowLightIntensity->onSave();
    ShadowShowGround->onSave();
    ShadowGroundBackFaceCull->onSave();
    ShadowGroundColor->onSave();
    ShadowGroundScale->onSave();
    ShadowGroundTransparency->onSave();
    ShadowGroundTexture->onSave();
    ShadowGroundTextureSize->onSave();
    ShadowGroundBumpMap->onSave();
    ShadowGroundShading->onSave();
    ShadowUpdateGround->onSave();
    ShadowDisplayMode->onSave();
    ShadowPrecision->onSave();
    ShadowSmoothBorder->onSave();
    ShadowSpreadSize->onSave();
    ShadowSpreadSampleSize->onSave();
    ShadowEpsilon->onSave();
    ShadowThreshold->onSave();
    ShadowBoundBoxScale->onSave();
    ShadowMaxDistance->onSave();
    ShadowTransparentShadow->onSave();
}

// Auto generated code (Tools/params_utils.py:517)
void DlgSettingsDrawStyles::loadSettings()
{
    // Auto generated code (Tools/params_utils.py:364)
    DefaultDrawStyle->onRestore();
    ForceSolidSingleSideLighting->onRestore();
    TransparencyOnTop->onRestore();
    SelectionLineThicken->onRestore();
    SelectionLineMaxWidth->onRestore();
    SelectionPointScale->onRestore();
    SelectionPointMaxSize->onRestore();
    SelectionLinePattern->onRestore();
    SelectionLinePatternScale->onRestore();
    SelectionHiddenLineWidth->onRestore();
    HiddenLineFaceColor->onRestore();
    HiddenLineOverrideFaceColor->onRestore();
    HiddenLineColor->onRestore();
    HiddenLineOverrideColor->onRestore();
    HiddenLineBackground->onRestore();
    HiddenLineOverrideBackground->onRestore();
    HiddenLineShaded->onRestore();
    HiddenLineShowOutline->onRestore();
    HiddenLinePerFaceOutline->onRestore();
    HiddenLineSceneOutline->onRestore();
    HiddenLineOutlineWidth->onRestore();
    HiddenLineHideFace->onRestore();
    HiddenLineHideSeam->onRestore();
    HiddenLineHideVertex->onRestore();
    HiddenLineTransparency->onRestore();
    HiddenLineOverrideTransparency->onRestore();
    HiddenLineWidth->onRestore();
    HiddenLinePointSize->onRestore();
    ShadowSpotLight->onRestore();
    ShadowLightColor->onRestore();
    ShadowLightIntensity->onRestore();
    ShadowShowGround->onRestore();
    ShadowGroundBackFaceCull->onRestore();
    ShadowGroundColor->onRestore();
    ShadowGroundScale->onRestore();
    ShadowGroundTransparency->onRestore();
    ShadowGroundTexture->onRestore();
    ShadowGroundTextureSize->onRestore();
    ShadowGroundBumpMap->onRestore();
    ShadowGroundShading->onRestore();
    ShadowUpdateGround->onRestore();
    ShadowDisplayMode->onRestore();
    ShadowPrecision->onRestore();
    ShadowSmoothBorder->onRestore();
    ShadowSpreadSize->onRestore();
    ShadowSpreadSampleSize->onRestore();
    ShadowEpsilon->onRestore();
    ShadowThreshold->onRestore();
    ShadowBoundBoxScale->onRestore();
    ShadowMaxDistance->onRestore();
    ShadowTransparentShadow->onRestore();
}

// Auto generated code (Tools/params_utils.py:524)
void DlgSettingsDrawStyles::retranslateUi()
{
    setWindowTitle(QObject::tr("Draw styles"));
    groupGeneral->setTitle(QObject::tr("General"));
    DefaultDrawStyle->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docDefaultDrawStyle()));
    labelDefaultDrawStyle->setText(QObject::tr("Default draw style"));
    labelDefaultDrawStyle->setToolTip(DefaultDrawStyle->toolTip());
    // Auto generated code (Tools/params_utils.py:895)
    DefaultDrawStyle->setItemText(0, QObject::tr("As Is"));
    DefaultDrawStyle->setItemData(0, QObject::tr("Draw style, normal display mode"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(1, QObject::tr("Points"));
    DefaultDrawStyle->setItemData(1, QObject::tr("Draw style, show points only"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(2, QObject::tr("Wireframe"));
    DefaultDrawStyle->setItemData(2, QObject::tr("Draw style, show wire frame only"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(3, QObject::tr("Hidden Line"));
    DefaultDrawStyle->setItemData(3, QObject::tr("Draw style, show hidden line by display object as transparent"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(4, QObject::tr("No Shading"));
    DefaultDrawStyle->setItemData(4, QObject::tr("Draw style, shading forced off"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(5, QObject::tr("Shaded"));
    DefaultDrawStyle->setItemData(5, QObject::tr("Draw style, shading force on"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(6, QObject::tr("Flat Lines"));
    DefaultDrawStyle->setItemData(6, QObject::tr("Draw style, show both wire frame and face with shading"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(7, QObject::tr("Tessellation"));
    DefaultDrawStyle->setItemData(7, QObject::tr("Draw style, show tessellation wire frame"), Qt::ToolTipRole);
    DefaultDrawStyle->setItemText(8, QObject::tr("Shadow"));
    DefaultDrawStyle->setItemData(8, QObject::tr("Draw style, drop shadows for the scene.\nClick this button while in shadow mode to toggle light manipulator"), Qt::ToolTipRole);
    ForceSolidSingleSideLighting->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docForceSolidSingleSideLighting()));
    ForceSolidSingleSideLighting->setText(QObject::tr("Force single side lighting on solid"));
    groupSelection->setTitle(QObject::tr("Selection"));
    TransparencyOnTop->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docTransparencyOnTop()));
    labelTransparencyOnTop->setText(QObject::tr("Transparency"));
    labelTransparencyOnTop->setToolTip(TransparencyOnTop->toolTip());
    SelectionLineThicken->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionLineThicken()));
    labelSelectionLineThicken->setText(QObject::tr("Line width multiplier"));
    labelSelectionLineThicken->setToolTip(SelectionLineThicken->toolTip());
    SelectionLineMaxWidth->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionLineMaxWidth()));
    labelSelectionLineMaxWidth->setText(QObject::tr("Maximum line width"));
    labelSelectionLineMaxWidth->setToolTip(SelectionLineMaxWidth->toolTip());
    SelectionPointScale->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionPointScale()));
    labelSelectionPointScale->setText(QObject::tr("Point size multiplier"));
    labelSelectionPointScale->setToolTip(SelectionPointScale->toolTip());
    SelectionPointMaxSize->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionPointMaxSize()));
    labelSelectionPointMaxSize->setText(QObject::tr("Maximum point size"));
    labelSelectionPointMaxSize->setToolTip(SelectionPointMaxSize->toolTip());
    SelectionLinePattern->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionLinePattern()));
    labelSelectionLinePattern->setText(QObject::tr("Selected hidden line pattern"));
    labelSelectionLinePattern->setToolTip(SelectionLinePattern->toolTip());
    SelectionLinePatternScale->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionLinePatternScale()));
    labelSelectionLinePatternScale->setText(QObject::tr("Selected line pattern scale"));
    labelSelectionLinePatternScale->setToolTip(SelectionLinePatternScale->toolTip());
    SelectionHiddenLineWidth->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docSelectionHiddenLineWidth()));
    labelSelectionHiddenLineWidth->setText(QObject::tr("Selected hidden line width"));
    labelSelectionHiddenLineWidth->setToolTip(SelectionHiddenLineWidth->toolTip());
    groupHiddenLines->setTitle(QObject::tr("Hidden Lines"));
    HiddenLineFaceColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineFaceColor()));
    HiddenLineOverrideFaceColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineOverrideFaceColor()));
    HiddenLineOverrideFaceColor->setText(QObject::tr("Override face color"));
    HiddenLineColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineColor()));
    HiddenLineOverrideColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineOverrideColor()));
    HiddenLineOverrideColor->setText(QObject::tr("Override line color"));
    HiddenLineBackground->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineBackground()));
    HiddenLineOverrideBackground->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineOverrideBackground()));
    HiddenLineOverrideBackground->setText(QObject::tr("Override background color"));
    HiddenLineShaded->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineShaded()));
    HiddenLineShaded->setText(QObject::tr("Shaded"));
    HiddenLineShowOutline->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineShowOutline()));
    HiddenLineShowOutline->setText(QObject::tr("Draw outline"));
    HiddenLinePerFaceOutline->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLinePerFaceOutline()));
    HiddenLinePerFaceOutline->setText(QObject::tr("Draw per face outline"));
    HiddenLineSceneOutline->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineSceneOutline()));
    HiddenLineSceneOutline->setText(QObject::tr("Draw scene outline"));
    HiddenLineOutlineWidth->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineOutlineWidth()));
    labelHiddenLineOutlineWidth->setText(QObject::tr("Outline width"));
    labelHiddenLineOutlineWidth->setToolTip(HiddenLineOutlineWidth->toolTip());
    HiddenLineHideFace->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineHideFace()));
    HiddenLineHideFace->setText(QObject::tr("Hide face"));
    HiddenLineHideSeam->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineHideSeam()));
    HiddenLineHideSeam->setText(QObject::tr("Hide seam edge"));
    HiddenLineHideVertex->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineHideVertex()));
    HiddenLineHideVertex->setText(QObject::tr("Hide vertex"));
    HiddenLineTransparency->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineTransparency()));
    HiddenLineOverrideTransparency->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineOverrideTransparency()));
    HiddenLineOverrideTransparency->setText(QObject::tr("Override transparency"));
    HiddenLineWidth->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLineWidth()));
    labelHiddenLineWidth->setText(QObject::tr("Line width"));
    labelHiddenLineWidth->setToolTip(HiddenLineWidth->toolTip());
    HiddenLinePointSize->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docHiddenLinePointSize()));
    labelHiddenLinePointSize->setText(QObject::tr("Point size"));
    labelHiddenLinePointSize->setToolTip(HiddenLinePointSize->toolTip());
    groupShadow->setTitle(QObject::tr("Shadow"));
    ShadowSpotLight->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowSpotLight()));
    ShadowSpotLight->setText(QObject::tr("Use spot light"));
    ShadowLightColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowLightColor()));
    labelShadowLightColor->setText(QObject::tr("Light color"));
    labelShadowLightColor->setToolTip(ShadowLightColor->toolTip());
    ShadowLightIntensity->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowLightIntensity()));
    labelShadowLightIntensity->setText(QObject::tr("Light intensity"));
    labelShadowLightIntensity->setToolTip(ShadowLightIntensity->toolTip());
    ShadowShowGround->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowShowGround()));
    ShadowShowGround->setText(QObject::tr("Show ground"));
    ShadowGroundBackFaceCull->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundBackFaceCull()));
    ShadowGroundBackFaceCull->setText(QObject::tr("Ground back face culling"));
    ShadowGroundColor->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundColor()));
    labelShadowGroundColor->setText(QObject::tr("Ground color"));
    labelShadowGroundColor->setToolTip(ShadowGroundColor->toolTip());
    ShadowGroundScale->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundScale()));
    labelShadowGroundScale->setText(QObject::tr("Ground scale"));
    labelShadowGroundScale->setToolTip(ShadowGroundScale->toolTip());
    ShadowGroundTransparency->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundTransparency()));
    labelShadowGroundTransparency->setText(QObject::tr("Ground transparency"));
    labelShadowGroundTransparency->setToolTip(ShadowGroundTransparency->toolTip());
    ShadowGroundTexture->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundTexture()));
    labelShadowGroundTexture->setText(QObject::tr("Ground texture"));
    labelShadowGroundTexture->setToolTip(ShadowGroundTexture->toolTip());
    ShadowGroundTextureSize->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundTextureSize()));
    labelShadowGroundTextureSize->setText(QObject::tr("Ground texture size"));
    labelShadowGroundTextureSize->setToolTip(ShadowGroundTextureSize->toolTip());
    ShadowGroundBumpMap->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundBumpMap()));
    labelShadowGroundBumpMap->setText(QObject::tr("Ground bump map"));
    labelShadowGroundBumpMap->setToolTip(ShadowGroundBumpMap->toolTip());
    ShadowGroundShading->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowGroundShading()));
    ShadowGroundShading->setText(QObject::tr("Ground shading"));
    ShadowUpdateGround->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowUpdateGround()));
    ShadowUpdateGround->setText(QObject::tr("Update ground on scene change"));
    ShadowDisplayMode->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowDisplayMode()));
    labelShadowDisplayMode->setText(QObject::tr("Override display mode"));
    labelShadowDisplayMode->setToolTip(ShadowDisplayMode->toolTip());
    // Auto generated code (Tools/params_utils.py:895)
    ShadowDisplayMode->setItemText(0, QObject::tr("Flat Lines"));
    ShadowDisplayMode->setItemText(1, QObject::tr("Shaded"));
    ShadowDisplayMode->setItemText(2, QObject::tr("As Is"));
    ShadowPrecision->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowPrecision()));
    labelShadowPrecision->setText(QObject::tr("Precision"));
    labelShadowPrecision->setToolTip(ShadowPrecision->toolTip());
    ShadowSmoothBorder->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowSmoothBorder()));
    labelShadowSmoothBorder->setText(QObject::tr("Smooth border"));
    labelShadowSmoothBorder->setToolTip(ShadowSmoothBorder->toolTip());
    ShadowSpreadSize->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowSpreadSize()));
    labelShadowSpreadSize->setText(QObject::tr("Spread size"));
    labelShadowSpreadSize->setToolTip(ShadowSpreadSize->toolTip());
    ShadowSpreadSampleSize->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowSpreadSampleSize()));
    labelShadowSpreadSampleSize->setText(QObject::tr("Spread sample size"));
    labelShadowSpreadSampleSize->setToolTip(ShadowSpreadSampleSize->toolTip());
    ShadowEpsilon->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowEpsilon()));
    labelShadowEpsilon->setText(QObject::tr("Epsilon"));
    labelShadowEpsilon->setToolTip(ShadowEpsilon->toolTip());
    ShadowThreshold->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowThreshold()));
    labelShadowThreshold->setText(QObject::tr("Threshold"));
    labelShadowThreshold->setToolTip(ShadowThreshold->toolTip());
    ShadowBoundBoxScale->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowBoundBoxScale()));
    labelShadowBoundBoxScale->setText(QObject::tr("Bounding box scale"));
    labelShadowBoundBoxScale->setToolTip(ShadowBoundBoxScale->toolTip());
    ShadowMaxDistance->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowMaxDistance()));
    labelShadowMaxDistance->setText(QObject::tr("Maximum distance"));
    labelShadowMaxDistance->setToolTip(ShadowMaxDistance->toolTip());
    ShadowTransparentShadow->setToolTip(QApplication::translate("ViewParams", Gui::ViewParams::docShadowTransparentShadow()));
    ShadowTransparentShadow->setText(QObject::tr("Transparent shadow"));
}

// Auto generated code (Tools/params_utils.py:537)
void DlgSettingsDrawStyles::changeEvent(QEvent *e)
{
    if (e->type() == QEvent::LanguageChange) {
        retranslateUi();
    }
    QWidget::changeEvent(e);
}

// Auto generated code (Tools/params_utils.py:548)
#include "moc_DlgSettingsDrawStyles.cpp"
//[[[end]]]
