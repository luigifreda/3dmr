/**
* This file is part of the ROS package path_planner which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SLIDER_BOX_CONNECTOR_H
#define SLIDER_BOX_CONNECTOR_H

#include <QObject>
#include <QSlider>
#include <QDoubleSpinBox>
#include <math.h>
#include <iostream>


///	\class BoxSliderManager
///	\author Luigi Freda
///	\brief This class connects together a QDoubleSpinBox and a QSlider so that the range of integer values of the slider is 1-1 through a scale factor.
///	\note
///	\date
///	\warning

class BoxSliderConnector : public QObject
{
    Q_OBJECT

public:

    /// main init, set box and slider
    void Init(QDoubleSpinBox* box, QSlider* slider)
    {
        slider_ = slider;
        box_ = box;

        connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(UpdateBox(int)));
        connect(box_, SIGNAL(valueChanged(double)), this, SLOT(UpdateSlider(double)));

        connect(box_, SIGNAL(valueChanged(double)), this, SIGNAL(ValueChanged(double)));

        value_ = 0.;
        boxStep_ = 1.;

    }

    /// initialize the box range and step
    void InitBox(double step, double minValue, double maxValue)
    {
        // store the old value
        double value = box_->value();

        boxStep_ = step;
        box_->setRange(minValue, maxValue);
        box_->setSingleStep(boxStep_);

        slider_->setRange(floor(minValue / boxStep_), ceil(maxValue / boxStep_));
        slider_->setSingleStep(1);

        // put back the old value (surprisingly, with the previous setRange() operations the box and the slider change their values)
        box_->setValue(value);
        slider_->setValue(lrint(value / boxStep_));

    }

    /// get box value

    double GetValue() const
    {
        return value_;
    }

Q_SIGNALS:

    double ValueChanged(double);

public Q_SLOTS:

    /// set box value  
    void SetValue(double value)
    {
        box_->setValue(value);
    }

protected Q_SLOTS: /// internal Q_SLOTS

    /// the box updates the slider
    void UpdateSlider(double value)
    {
        if (value_ != value)
        {
            value_ = value;
            slider_->setValue(lrint(value / boxStep_));
        }
    }

    /// the slider updates the box

    void UpdateBox(int value)
    {
        box_->setValue(value * boxStep_);
    }


protected:

    QSlider* slider_;
    QDoubleSpinBox* box_;

    double boxStep_;
    double value_;

};

#endif
