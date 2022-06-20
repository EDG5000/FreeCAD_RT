/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <FreeCAD@juergen-riegel.net>        *
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
# include <sstream>
# include <QRegExp>
# include <QTextStream>
# include <Precision.hxx>
#endif

#include "ui_TaskPadParameters.h"
#include "TaskPadParameters.h"
#include <App/Application.h>
#include <App/Document.h>
#include <Base/UnitsApi.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/BitmapFactory.h>
#include <Gui/ViewProvider.h>
#include <Gui/WaitCursor.h>
#include <Base/Console.h>
#include <Gui/Selection.h>
#include <Gui/Command.h>
#include <Mod/PartDesign/App/FeaturePad.h>
#include <Mod/PartDesign/App/FeatureExtrusion.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include "TaskSketchBasedParameters.h"
#include "ReferenceSelection.h"
#include "ViewProviderExtrusion.h"
#include "Utils.h"

using namespace PartDesignGui;
using namespace Gui;

/* TRANSLATOR PartDesignGui::TaskPadParameters */

TaskPadParameters::TaskPadParameters(ViewProviderPad *PadView, QWidget *parent, bool newObj)
    : TaskSketchBasedParameters(PadView, parent, "PartDesign_Pad",tr("Pad parameters")), useElement(false)
{
    setupUI(newObj);
}

TaskPadParameters::TaskPadParameters(ViewProviderPad *PadView, QWidget *parent, bool newObj,
                                     const std::string& pixmapname, const QString& parname)
    : TaskSketchBasedParameters(PadView, parent, pixmapname, parname), useElement(true)
{
    setupUI(newObj);
}

void TaskPadParameters::setupUI(bool newObj)
{
    // we need a separate container widget to add all controls to
    proxy = new QWidget(this);
    ui.reset(new Ui_TaskPadParameters());
    ui->setupUi(proxy);
    if (useElement) {
        ui->buttonFace->setText(tr("Element"));
#if QT_VERSION >= 0x040700
        ui->lineFaceName->setPlaceholderText(tr("No element selected"));
#endif
    }
    else {
#if QT_VERSION >= 0x040700
        ui->lineFaceName->setPlaceholderText(tr("No face selected"));
#endif
    }
    addBlinkEditor(ui->lineFaceName);

    ui->lineFaceName->installEventFilter(this);
    ui->lineFaceName->setMouseTracking(true);

    this->initUI(proxy);
    this->groupLayout()->addWidget(proxy);

    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());

    // set the history path
    ui->lengthEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/PadLength"));
    ui->lengthEdit2->setParamGrpPath(QByteArray("User parameter:BaseApp/History/PadLength2"));
    ui->offsetEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/PadOffset"));
    ui->taperAngleEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/TaperAngle"));
    ui->taperAngleEdit2->setParamGrpPath(QByteArray("User parameter:BaseApp/History/TaperAngle2"));
    ui->innerTaperAngleEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/InnerTaperAngle"));
    ui->innerTaperAngleEdit2->setParamGrpPath(QByteArray("User parameter:BaseApp/History/InnerTaperAngle2"));

    ui->taperAngleEdit->setToolTip(QApplication::translate(
                "Property", pcPad->TaperAngle.getDocumentation()));
    ui->taperAngleEdit2->setToolTip(QApplication::translate(
                "Property", pcPad->TaperAngleRev.getDocumentation()));
    ui->innerTaperAngleEdit->setToolTip(QApplication::translate(
                "Property", pcPad->InnerTaperAngle.getDocumentation()));
    ui->innerTaperAngleEdit2->setToolTip(QApplication::translate(
                "Property", pcPad->InnerTaperAngleRev.getDocumentation()));

    // Bind input fields to properties
    ui->lengthEdit->bind(pcPad->Length);
    ui->lengthEdit2->bind(pcPad->Length2);
    ui->taperAngleEdit->bind(pcPad->TaperAngle);
    ui->taperAngleEdit2->bind(pcPad->TaperAngleRev);
    ui->innerTaperAngleEdit->bind(pcPad->InnerTaperAngle);
    ui->innerTaperAngleEdit2->bind(pcPad->InnerTaperAngleRev);

    ui->XDirectionEdit->bind(App::ObjectIdentifier::parse(pcPad, std::string(".Direction.x")));
    ui->YDirectionEdit->bind(App::ObjectIdentifier::parse(pcPad, std::string(".Direction.y")));
    ui->ZDirectionEdit->bind(App::ObjectIdentifier::parse(pcPad, std::string(".Direction.z")));

    ui->offsetEdit->bind(pcPad->Offset);

    // set decimals for the direction edits
    int UserDecimals = Base::UnitsApi::getDecimals();
    ui->XDirectionEdit->setDecimals(UserDecimals);
    ui->YDirectionEdit->setDecimals(UserDecimals);
    ui->ZDirectionEdit->setDecimals(UserDecimals);

    ui->changeMode->clear();
    ui->changeMode->insertItem(0, tr("Dimension"));
    ui->changeMode->insertItem(1, tr("To last"));
    ui->changeMode->insertItem(2, tr("To first"));
    if (useElement)
        ui->changeMode->insertItem(3, tr("Up to Element"));
    else
        ui->changeMode->insertItem(3, tr("Up to face"));
    ui->changeMode->insertItem(4, tr("Two dimensions"));

    QMetaObject::connectSlotsByName(this);

    connect(ui->lengthEdit, SIGNAL(valueChanged(double)),
            this, SLOT(onLengthChanged(double)));
    connect(ui->lengthEdit2, SIGNAL(valueChanged(double)),
            this, SLOT(onLength2Changed(double)));
    connect(ui->taperAngleEdit, SIGNAL(valueChanged(double)),
            this, SLOT(onAngleChanged(double)));
    connect(ui->taperAngleEdit2, SIGNAL(valueChanged(double)),
            this, SLOT(onAngle2Changed(double)));
    connect(ui->innerTaperAngleEdit, SIGNAL(valueChanged(double)),
            this, SLOT(onInnerAngleChanged(double)));
    connect(ui->innerTaperAngleEdit2, SIGNAL(valueChanged(double)),
            this, SLOT(onInnerAngle2Changed(double)));
    connect(ui->groupBoxDirection, SIGNAL(toggled(bool)),
        this, SLOT(onGBDirectionChanged(bool)));
    connect(ui->XDirectionEdit, SIGNAL(valueChanged(double)),
        this, SLOT(onXDirectionEditChanged(double)));
    connect(ui->YDirectionEdit, SIGNAL(valueChanged(double)),
        this, SLOT(onYDirectionEditChanged(double)));
    connect(ui->ZDirectionEdit, SIGNAL(valueChanged(double)),
        this, SLOT(onZDirectionEditChanged(double)));
    connect(ui->offsetEdit, SIGNAL(valueChanged(double)),
            this, SLOT(onOffsetChanged(double)));
    connect(ui->checkBoxMidplane, SIGNAL(toggled(bool)),
            this, SLOT(onMidplaneChanged(bool)));
    connect(ui->checkBoxReversed, SIGNAL(toggled(bool)),
            this, SLOT(onReversedChanged(bool)));
    connect(ui->checkBoxUsePipe, SIGNAL(toggled(bool)),
            this, SLOT(onUsePipeChanged(bool)));
    connect(ui->changeMode, SIGNAL(currentIndexChanged(int)),
            this, SLOT(onModeChanged(int)));
    connect(ui->buttonFace, SIGNAL(clicked()),
            this, SLOT(onButtonFace()));
    connect(ui->lineFaceName, SIGNAL(textEdited(QString)),
            this, SLOT(onFaceName(QString)));

    refresh();

    // if it is a newly created object use the last value of the history
    // TODO: newObj doesn't supplied normally by any caller (2015-07-24, Fat-Zer)
    if(newObj){
        ui->lengthEdit->setToLastUsedValue();
        ui->lengthEdit->selectNumber();
        ui->lengthEdit2->setToLastUsedValue();
        ui->lengthEdit2->selectNumber();
        ui->offsetEdit->setToLastUsedValue();
        ui->offsetEdit->selectNumber();
        ui->taperAngleEdit->setToLastUsedValue();
        ui->taperAngleEdit->selectNumber();
        ui->taperAngleEdit2->setToLastUsedValue();
        ui->taperAngleEdit2->selectNumber();
        ui->innerTaperAngleEdit->setToLastUsedValue();
        ui->innerTaperAngleEdit->selectNumber();
        ui->innerTaperAngleEdit2->setToLastUsedValue();
        ui->innerTaperAngleEdit2->selectNumber();
    }
}

void TaskPadParameters::refresh()
{
    if (!vp || !vp->getObject())
        return;

    // Get the feature data
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    Base::Quantity l = pcPad->Length.getQuantityValue();
    Base::Quantity l2 = pcPad->Length2.getQuantityValue();
    bool useCustom = pcPad->UseCustomVector.getValue();
    double xs = pcPad->Direction.getValue().x;
    double ys = pcPad->Direction.getValue().y;
    double zs = pcPad->Direction.getValue().z;
    Base::Quantity off = pcPad->Offset.getQuantityValue();
    bool midplane = pcPad->Midplane.getValue();
    bool reversed = pcPad->Reversed.getValue();
    int index = pcPad->Type.getValue(); // must extract value here, clear() kills it!
    double angle = pcPad->TaperAngle.getValue();
    double angle2 = pcPad->TaperAngleRev.getValue();
    double innerAngle = pcPad->InnerTaperAngle.getValue();
    double innerAngle2 = pcPad->InnerTaperAngleRev.getValue();

    // Temporarily prevent unnecessary feature recomputes
    for (QWidget* child : proxy->findChildren<QWidget*>())
        child->blockSignals(true);

    // Fill data into dialog elements
    ui->lengthEdit->setValue(l);
    ui->lengthEdit2->setValue(l2);
    ui->groupBoxDirection->setChecked(useCustom);
    ui->XDirectionEdit->setValue(xs);
    ui->YDirectionEdit->setValue(ys);
    ui->ZDirectionEdit->setValue(zs);
    ui->offsetEdit->setValue(off);
    ui->taperAngleEdit->setValue(angle);
    ui->taperAngleEdit2->setValue(angle2);
    ui->innerTaperAngleEdit->setValue(innerAngle);
    ui->innerTaperAngleEdit2->setValue(innerAngle2);

    ui->checkBoxMidplane->setChecked(midplane);
    // According to bug #0000521 the reversed option
    // shouldn't be de-activated if the pad has a support face
    ui->checkBoxReversed->setChecked(reversed);

    ui->checkBoxUsePipe->setChecked(pcPad->UsePipeForDraft.getValue());

    // Set object labels
    App::DocumentObject* obj = pcPad->UpToFace.getValue();
    std::vector<std::string> subStrings = pcPad->UpToFace.getSubValues(false);
    if (obj && (subStrings.empty() || subStrings.front().empty())) {
        ui->lineFaceName->setText(QString::fromUtf8(obj->Label.getValue()));
        ui->lineFaceName->setProperty("FeatureName", QByteArray(obj->getNameInDocument()));
        ui->lineFaceName->setProperty("FaceName", QVariant());
    }
    else if (obj) {
        ui->lineFaceName->setText(QString::fromLatin1("%1:%2")
                                  .arg(QString::fromUtf8(obj->Label.getValue()))
                                  .arg(QString::fromLatin1(subStrings.front().c_str())));
        ui->lineFaceName->setProperty("FeatureName", QByteArray(obj->getNameInDocument()));
        ui->lineFaceName->setProperty("FaceName", QByteArray(subStrings.front().c_str()));

    }
    else {
        ui->lineFaceName->clear();
        ui->lineFaceName->setProperty("FeatureName", QVariant());
        ui->lineFaceName->setProperty("FaceName", QVariant());
    }

    ui->changeMode->setCurrentIndex(index);

    // Temporarily prevent unnecessary feature recomputes
    for (QWidget* child : proxy->findChildren<QWidget*>())
        child->blockSignals(false);

    updateUI(index);
    TaskSketchBasedParameters::refresh();
}

void TaskPadParameters::updateUI(int index)
{
    // disable/hide everything unless we are sure we don't need it
    // exception: the direction parameters are in any case visible
    bool isLengthEditVisable  = false;
    bool isLengthEdit2Visable = false;
    bool isOffsetEditVisable  = false;
    bool isMidplateEnabled    = false;
    bool isReversedEnabled    = false;
    bool isFaceEditEnabled    = false;

    // dimension
    if (index == 0) {
        isLengthEditVisable = true;
        ui->lengthEdit->selectNumber();
        // Make sure that the spin box has the focus to get key events
        // Calling setFocus() directly doesn't work because the spin box is not
        // yet visible.
        QMetaObject::invokeMethod(ui->lengthEdit, "setFocus", Qt::QueuedConnection);
        isMidplateEnabled = true;
        // Reverse only makes sense if Midplane is not true
        isReversedEnabled = !ui->checkBoxMidplane->isChecked();
    }
    // up to first/last
    else if (index == 1 || index == 2) {
        isOffsetEditVisable  = true;
        isReversedEnabled = true;
    }
    // up to face
    else if (index == 3) {
        isOffsetEditVisable  = true;
        isFaceEditEnabled    = true;
        // Go into reference selection mode if no face has been selected yet
        if (ui->lineFaceName->property("FeatureName").isNull())
            onButtonFace(true);
        ui->lineFaceName->show();
        ui->buttonFace->show();
    }
    // two dimensions
    else {
        isLengthEditVisable = true;
        isLengthEdit2Visable = true;
    }

    if (index != 3) {
        ui->lineFaceName->hide();
        ui->buttonFace->hide();
    }

    ui->lengthEdit->setVisible( isLengthEditVisable );
    ui->lengthEdit->setEnabled( isLengthEditVisable );
    ui->labelLength->setVisible( isLengthEditVisable );

    ui->offsetEdit->setVisible( isOffsetEditVisable );
    ui->offsetEdit->setEnabled( isOffsetEditVisable );
    ui->labelOffset->setVisible( isOffsetEditVisable );

    ui->checkBoxMidplane->setEnabled( isMidplateEnabled );

    ui->checkBoxReversed->setEnabled( isReversedEnabled );

    ui->lengthEdit2->setVisible( isLengthEdit2Visable );
    ui->lengthEdit2->setEnabled( isLengthEdit2Visable );
    ui->labelLength2->setVisible( isLengthEdit2Visable );

    bool angleVisible = index == 0 || index == 4;
    ui->taperAngleEdit->setVisible( angleVisible );
    ui->taperAngleEdit->setEnabled( angleVisible );
    ui->labelTaperAngle->setVisible( angleVisible );
    ui->taperAngleEdit2->setVisible( angleVisible );
    ui->taperAngleEdit2->setEnabled( angleVisible );
    ui->labelTaperAngle2->setVisible( angleVisible );
    ui->innerTaperAngleEdit->setVisible( angleVisible );
    ui->innerTaperAngleEdit->setEnabled( angleVisible );
    ui->labelInnerTaperAngle->setVisible( angleVisible );
    ui->innerTaperAngleEdit2->setVisible( angleVisible );
    ui->innerTaperAngleEdit2->setEnabled( angleVisible );
    ui->labelInnerTaperAngle2->setVisible( angleVisible );

    ui->buttonFace->setEnabled( isFaceEditEnabled );
    ui->lineFaceName->setEnabled( isFaceEditEnabled );
    if (!isFaceEditEnabled) {
        onButtonFace(false);
    }
}

void TaskPadParameters::onSelectionChanged(const Gui::SelectionChanges& msg)
{
    if (msg.Type == Gui::SelectionChanges::AddSelection) {
        QString refText = onAddSelection(msg);
        if (refText.length() > 0) {
            ui->lineFaceName->blockSignals(true);
            ui->lineFaceName->setText(refText);
            QStringList list(refText.split(QLatin1Char(':')));
            ui->lineFaceName->setProperty("FeatureName", list[0].toUtf8());
            ui->lineFaceName->setProperty("FaceName", list.size()>1 ? list[1].toUtf8() : QByteArray());
            ui->lineFaceName->blockSignals(false);
            // Turn off reference selection mode
            onButtonFace(false);
        } else {
            ui->lineFaceName->blockSignals(true);
            ui->lineFaceName->clear();
            ui->lineFaceName->setProperty("FeatureName", QVariant());
            ui->lineFaceName->setProperty("FaceName", QVariant());
            ui->lineFaceName->blockSignals(false);
        }
    } else if (msg.Type == Gui::SelectionChanges::ClrSelection) {
        ui->lineFaceName->blockSignals(true);
        ui->lineFaceName->clear();
        ui->lineFaceName->setProperty("FeatureName", QVariant());
        ui->lineFaceName->setProperty("FaceName", QVariant());
        ui->lineFaceName->blockSignals(false);
    }
}

void TaskPadParameters::onLengthChanged(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Length.setValue(len);
    recomputeFeature();
}

void TaskPadParameters::onLength2Changed(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Length2.setValue(len);
    recomputeFeature();
}

void TaskPadParameters::onAngleChanged(double angle)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->TaperAngle.setValue(angle);
    recomputeFeature();
}

void TaskPadParameters::onAngle2Changed(double angle)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->TaperAngleRev.setValue(angle);
    recomputeFeature();
}

void TaskPadParameters::onInnerAngleChanged(double angle)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->InnerTaperAngle.setValue(angle);
    recomputeFeature();
}

void TaskPadParameters::onInnerAngle2Changed(double angle)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->InnerTaperAngleRev.setValue(angle);
    recomputeFeature();
}

void TaskPadParameters::onGBDirectionChanged(bool on)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->UseCustomVector.setValue(on);
    recomputeFeature();
}

void TaskPadParameters::onXDirectionEditChanged(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Direction.setValue(len, pcPad->Direction.getValue().y, pcPad->Direction.getValue().z);
    recomputeFeature();
    // checking for case of a null vector is done in FeaturePad.cpp
    // if there was a null vector, the normal vector of the sketch is used.
    // therefore the vector component edits must be updated
    updateDirectionEdits();
}


void TaskPadParameters::onYDirectionEditChanged(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Direction.setValue(pcPad->Direction.getValue().x, len, pcPad->Direction.getValue().z);
    recomputeFeature();
    updateDirectionEdits();
}

void TaskPadParameters::onZDirectionEditChanged(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Direction.setValue(pcPad->Direction.getValue().x, pcPad->Direction.getValue().y, len);
    recomputeFeature();
    updateDirectionEdits();
}

void TaskPadParameters::updateDirectionEdits(void)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    ui->XDirectionEdit->setValue(pcPad->Direction.getValue().x);
    ui->YDirectionEdit->setValue(pcPad->Direction.getValue().y);
    ui->ZDirectionEdit->setValue(pcPad->Direction.getValue().z);
}

void TaskPadParameters::onOffsetChanged(double len)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Offset.setValue(len);
    recomputeFeature();
}

void TaskPadParameters::onMidplaneChanged(bool on)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Midplane.setValue(on);
    ui->checkBoxReversed->setEnabled(!on);
    recomputeFeature();
}

void TaskPadParameters::onUsePipeChanged(bool on)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->UsePipeForDraft.setValue(on);
    ui->checkBoxReversed->setEnabled(!on);
    recomputeFeature();
}

void TaskPadParameters::onReversedChanged(bool on)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());
    pcPad->Reversed.setValue(on);
    recomputeFeature();
}

void TaskPadParameters::onModeChanged(int index)
{
    PartDesign::Pad* pcPad = static_cast<PartDesign::Pad*>(vp->getObject());

    switch (index) {
        case 0:
            pcPad->Type.setValue("Length");
            // Avoid error message
            if (ui->lengthEdit->value() < Base::Quantity(Precision::Confusion(), Base::Unit::Length))
                ui->lengthEdit->setValue(5.0);
            break;
        case 1: pcPad->Type.setValue("UpToLast"); break;
        case 2: pcPad->Type.setValue("UpToFirst"); break;
        case 3: pcPad->Type.setValue("UpToFace"); break;
        default: pcPad->Type.setValue("TwoLengths");
    }

    updateUI(index);
    if (index != 3 || pcPad->UpToFace.getValue())
        recomputeFeature();
}

void TaskPadParameters::onButtonFace(const bool pressed)
{
    this->blockConnection(!pressed);

    if (vp && vp->getObject()
           && vp->getObject()->isDerivedFrom(PartDesign::Extrusion::getClassTypeId()))
    {
        if (pressed) {
            Gui::Selection().clearSelection();
            Gui::Selection().addSelectionGate(
                    new ReferenceSelection(vp->getObject(), true, true, false, true));
        }
        else
            Gui::Selection().rmvSelectionGate();
    }
    else
        TaskSketchBasedParameters::onSelectReference(pressed, false, true, false);

    // Update button if onButtonFace() is called explicitly
    ui->buttonFace->setChecked(pressed);
}

void TaskPadParameters::onFaceName(const QString& text)
{
    if (text.isEmpty()) {
        // if user cleared the text field then also clear the properties
        ui->lineFaceName->setProperty("FeatureName", QVariant());
        ui->lineFaceName->setProperty("FaceName", QVariant());
    }
    else {
        // expect that the label of an object is used
        QStringList parts = text.split(QChar::fromLatin1(':'));
        QString label = parts[0];
        QVariant name = objectNameByLabel(label, ui->lineFaceName->property("FeatureName"));
        if (name.isValid()) {
            parts[0] = name.toString();
            QString uptoface = parts.join(QString::fromLatin1(":"));
            ui->lineFaceName->setProperty("FeatureName", name);
            ui->lineFaceName->setProperty("FaceName", setUpToFace(uptoface));
        }
        else {
            ui->lineFaceName->setProperty("FeatureName", QVariant());
            ui->lineFaceName->setProperty("FaceName", QVariant());
        }
    }
}

double TaskPadParameters::getLength(void) const
{
    return ui->lengthEdit->value().getValue();
}

double TaskPadParameters::getLength2(void) const
{
    return ui->lengthEdit2->value().getValue();
}

bool   TaskPadParameters::getCustom(void) const
{
    return ui->groupBoxDirection->isChecked();
}

double TaskPadParameters::getXDirection(void) const
{
    return ui->XDirectionEdit->value();
}

double TaskPadParameters::getYDirection(void) const
{
    return ui->YDirectionEdit->value();
}

double TaskPadParameters::getZDirection(void) const
{
    return ui->ZDirectionEdit->value();
}

double TaskPadParameters::getOffset(void) const
{
    return ui->offsetEdit->value().getValue();
}

bool   TaskPadParameters::getReversed(void) const
{
    return ui->checkBoxReversed->isChecked();
}

bool   TaskPadParameters::getMidplane(void) const
{
    return ui->checkBoxMidplane->isChecked();
}

int TaskPadParameters::getMode(void) const
{
    return ui->changeMode->currentIndex();
}

QString TaskPadParameters::getFaceName(void) const
{
    // 'Up to face' mode
    if (getMode() == 3) {
        QVariant featureName = ui->lineFaceName->property("FeatureName");
        if (featureName.isValid()) {
            QString faceName = ui->lineFaceName->property("FaceName").toString();
            return getFaceReference(featureName.toString(), faceName);
        }
    }
    return QString::fromLatin1("None");
}

bool TaskPadParameters::eventFilter(QObject *o, QEvent *ev)
{
    switch(ev->type()) {
    case QEvent::Leave:
        Gui::Selection().rmvPreselect();
        break;
    case QEvent::Enter:
        if (vp && o == ui->lineFaceName) {
            auto pad = static_cast<PartDesign::Pad*>(vp->getObject());
            auto obj = pad->UpToFace.getValue();
            if (obj) {
                const auto &subs = pad->UpToFace.getSubValues(true);
                PartDesignGui::highlightObjectOnTop(
                        App::SubObjectT(obj, subs.size()?subs[0].c_str():""));
            }
        }
        break;
    default:
        break;
    }
    return false;
}


TaskPadParameters::~TaskPadParameters()
{
}

void TaskPadParameters::changeEvent(QEvent *e)
{
    TaskBox::changeEvent(e);
    if (e->type() == QEvent::LanguageChange) {
        ui->lengthEdit->blockSignals(true);
        ui->lengthEdit2->blockSignals(true);
        ui->XDirectionEdit->blockSignals(true);
        ui->YDirectionEdit->blockSignals(true);
        ui->ZDirectionEdit->blockSignals(true);
        ui->offsetEdit->blockSignals(true);
        ui->lineFaceName->blockSignals(true);
        ui->changeMode->blockSignals(true);
        int index = ui->changeMode->currentIndex();
        ui->retranslateUi(proxy);
        ui->changeMode->clear();
        ui->changeMode->addItem(tr("Dimension"));
        ui->changeMode->addItem(tr("To last"));
        ui->changeMode->addItem(tr("To first"));
        if (useElement)
            ui->changeMode->addItem(tr("Up to Element"));
        else
            ui->changeMode->addItem(tr("Up to face"));
        ui->changeMode->addItem(tr("Two dimensions"));
        ui->changeMode->setCurrentIndex(index);

        if (useElement) {
            ui->buttonFace->setText(tr("Element"));
#if QT_VERSION >= 0x040700
            ui->lineFaceName->setPlaceholderText(tr("No element selected"));
#endif
        }
        else {
#if QT_VERSION >= 0x040700
            ui->lineFaceName->setPlaceholderText(tr("No face selected"));
#endif
        }
        addBlinkEditor(ui->lineFaceName);

        ui->lengthEdit->blockSignals(false);
        ui->lengthEdit2->blockSignals(false);
        ui->XDirectionEdit->blockSignals(false);
        ui->YDirectionEdit->blockSignals(false);
        ui->ZDirectionEdit->blockSignals(false);
        ui->offsetEdit->blockSignals(false);
        ui->lineFaceName->blockSignals(false);
        ui->changeMode->blockSignals(false);
    }
}

void TaskPadParameters::saveHistory(void)
{
    // save the user values to history
    ui->lengthEdit->pushToHistory();
    ui->lengthEdit2->pushToHistory();
    ui->offsetEdit->pushToHistory();
    ui->taperAngleEdit->pushToHistory();
    ui->taperAngleEdit2->pushToHistory();
    ui->innerTaperAngleEdit->pushToHistory();
    ui->innerTaperAngleEdit2->pushToHistory();

    TaskSketchBasedParameters::saveHistory();
}

void TaskPadParameters::apply()
{
    auto obj = vp->getObject();

    ui->lengthEdit->apply();
    ui->lengthEdit2->apply();
    FCMD_OBJ_CMD(obj, "UseCustomVector = " << (getCustom() ? 1 : 0));
    FCMD_OBJ_CMD(obj, "Direction = ("
        << getXDirection() << ", " << getYDirection() << ", " << getZDirection() << ")");
    FCMD_OBJ_CMD(obj,"Type = " << getMode());
    QString facename = getFaceName();
    FCMD_OBJ_CMD(obj,"UpToFace = " << facename.toLatin1().data());
    FCMD_OBJ_CMD(obj,"Reversed = " << (getReversed()?1:0));
    FCMD_OBJ_CMD(obj,"Midplane = " << (getMidplane()?1:0));
    FCMD_OBJ_CMD(obj,"Offset = " << getOffset());
}

//**************************************************************************
//**************************************************************************
// TaskDialog
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

TaskDlgPadParameters::TaskDlgPadParameters(ViewProviderPad *PadView, bool /*newObj*/)
    : TaskDlgSketchBasedParameters(PadView)
{
    assert(vp);
    Content.push_back ( new TaskPadParameters(PadView ) );
}

TaskDlgPadParameters::TaskDlgPadParameters(ViewProviderPad *PadView, bool newObj,
                                           const std::string& pixmapname, const QString& parname)
    : TaskDlgSketchBasedParameters(PadView)
{
    assert(vp);
    Content.push_back ( new TaskPadParameters(PadView, nullptr, newObj, pixmapname, parname ) );
}

bool TaskDlgPadParameters::accept() {
    if (vp && vp->isDerivedFrom(ViewProviderExtrusion::getClassTypeId()))
        return TaskDlgFeatureParameters::accept();
    return TaskDlgSketchBasedParameters::accept();
}

//==== calls from the TaskView ===============================================================

#include "moc_TaskPadParameters.cpp"
