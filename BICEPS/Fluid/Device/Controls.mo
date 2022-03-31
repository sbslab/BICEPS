within BICEPS.Fluid.Device;
package Controls "Controls for equipment subsystems"
  model HeatPump "Heat pump control"
    extends Modelica.Blocks.Icons.Block;
    parameter Boolean biomimeticControl = true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
    parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
    parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
    parameter Modelica.SIunits.Time riseTime=1
      "Rise time of the filter (time to reach 99.6 % of the transition speed)";
    parameter Modelica.SIunits.Temperature THeaWatSup_nominal=313.15
      "Heating water supply temperature"
      annotation (Dialog(group="Nominal condition"));
    Modelica.Blocks.Interfaces.RealOutput TSet(
      final quantity="ThermodynamicTemperature",
      final unit="K") "Setpoint temperature"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Modelica.Blocks.Interfaces.RealInput TConEnt(
      final quantity="ThermodynamicTemperature",
      final unit="K") "Setpoint temperatureMeasured entering condenser water temperature"
      annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
    Buildings.Controls.OBC.CDL.Logical.Switch enaHeaPum(u2(start=false))
      "Enable heat pump by switching to actual set point"
      annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    Modelica.Blocks.Interfaces.BooleanInput uEna "Enable signal"
      annotation (Placement(transformation(extent={{-140,-40},{-100,0}})));
    Utilities.Math.CubicHermiteInverse spl(
      final xMin=TMin,
      final xMax=TMax,
      final x0=T0) if biomimeticControl
      annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
    Buildings.Controls.OBC.CDL.Continuous.Limiter lim(
      final uMax=TMax,
      final uMin=TMin) if biomimeticControl
      annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
    Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
      "Control signal"
      annotation (Placement(transformation(extent={{-140,20},{-100,60}}),
          iconTransformation(extent={{-140,20},{-100,60}})));
    Modelica.Blocks.Continuous.Filter fil(
      analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
      filterType=Modelica.Blocks.Types.FilterType.LowPass,
      order=2,
      f_cut=5/(2*Modelica.Constants.pi*riseTime),
      init=Modelica.Blocks.Types.Init.InitialOutput,
      y_start=THeaWatSup_nominal)
      "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
      annotation (Placement(transformation(extent={{72,-10},{92,10}})));
    Modelica.Blocks.Interfaces.RealInput TSetSta(
      final quantity="ThermodynamicTemperature",
      final unit="K") if not biomimeticControl
      "Static temperature setpoint if normal control"
      annotation (Placement(transformation(extent={{-140,80},{-100,120}}),
          iconTransformation(extent={{-140,80},{-100,120}})));
  equation
    connect(spl.x,lim. u)
      annotation (Line(points={{-59,40},{-42,40}},
                                               color={0,0,127}));
    connect(uEna, enaHeaPum.u2)
      annotation (Line(points={{-120,-20},{-42,-20},{-42,0},{38,0}},
                                                 color={255,0,255}));
    connect(TConEnt, enaHeaPum.u3) annotation (Line(points={{-120,-80},{0,-80},{0,
            -8},{38,-8}}, color={0,0,127}));
    connect(lim.y, enaHeaPum.u1) annotation (Line(points={{-18,40},{0,40},{0,8},{38,
            8}},                    color={0,0,127}));
    connect(y, spl.y)
      annotation (Line(points={{-120,40},{-102,40},{-102,40},{-82,40}},
                                                    color={0,0,127}));
    connect(enaHeaPum.y, fil.u)
      annotation (Line(points={{62,0},{70,0}}, color={0,0,127}));
    connect(fil.y, TSet)
      annotation (Line(points={{93,0},{98,0},{98,0},{110,0}}, color={0,0,127}));
    connect(TSetSta, enaHeaPum.u1) annotation (Line(points={{-120,100},{20,100},{
            20,8},{38,8}},
                        color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end HeatPump;

  block PrimaryVariableFlow
    "Ideal control of condenser or evaporator variable flow rate"
    extends Modelica.Blocks.Icons.Block;
    parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
      "Heat flow rate at nominal conditions (>0 for condenser)";
    parameter Modelica.SIunits.TemperatureDifference dT_nominal(
      min=if Q_flow_nominal>0 then Modelica.Constants.eps else -100,
      max=if Q_flow_nominal<0 then -Modelica.Constants.eps else 100)
      "DeltaT at nominal conditions (>0 for condenser)";
    parameter Real ratFloMin(
      final unit="1",
      final min=0,
      final max=1)=0.3
      "Minimum mass flow rate (ratio to nominal)";
    constant Modelica.SIunits.SpecificHeatCapacity cp=
      Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
      "Specific heat capacity";
    final parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)=
      Q_flow_nominal/cp/dT_nominal
      "Mass flow rate at nominal conditions";
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput m_flow(final unit="kg/s")
      "Mass flow rate"
      annotation (Placement(transformation(extent={{100,-20},{140,20}}),
        iconTransformation(extent={{100,-20},{140,20}})));
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant masFloMin(
      final k=ratFloMin*m_flow_nominal)
      "Minimum mass flow rate"
      annotation (Placement(transformation(extent={{-30,30},{-10,50}})));
    Buildings.Controls.OBC.CDL.Continuous.Gain masFlo_dT(
      final k=1/cp/dT_nominal)
      "Mass flow rate for constant DeltaT"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Buildings.Controls.OBC.CDL.Continuous.Max masFlo "Mass flow rate"
      annotation (Placement(transformation(extent={{20,-10},{40,10}})));
    Buildings.Controls.OBC.CDL.Continuous.Abs abs1 "Absolute value"
      annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
    Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
      annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
    Buildings.Controls.OBC.CDL.Continuous.Product flo
      "Zero flow rate if not enabled"
      annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    Modelica.Blocks.Interfaces.BooleanInput u "Enable signal"
      annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
    Modelica.Blocks.Interfaces.RealInput loa "Load"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  equation
    connect(masFloMin.y, masFlo.u1) annotation (Line(points={{-8,40},{0,40},{0,6},
            {18,6}},    color={0,0,127}));
    connect(masFlo_dT.y, abs1.u)
      annotation (Line(points={{-58,0},{-32,0}}, color={0,0,127}));
    connect(abs1.y, masFlo.u2)
      annotation (Line(points={{-8,0},{0,0},{0,-6},{18,-6}},   color={0,0,127}));
    connect(masFlo.y, flo.u2)
      annotation (Line(points={{42,0},{50,0},{50,-6},{58,-6}}, color={0,0,127}));
    connect(booToRea.y, flo.u1) annotation (Line(points={{-58,80},{50,80},{50,6},{
            58,6}}, color={0,0,127}));
    connect(flo.y, m_flow)
      annotation (Line(points={{82,0},{120,0}}, color={0,0,127}));
    connect(booToRea.u, u)
      annotation (Line(points={{-82,80},{-120,80}}, color={255,0,255}));
    connect(masFlo_dT.u, loa)
      annotation (Line(points={{-82,0},{-120,0}}, color={0,0,127}));
    annotation (
      defaultComponentName="conFlo",
      Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>
This block implements an ideal control of the evaporator (or condenser) water
mass flow rate.
The control intent aims to maintain a constant water temperature difference
<code>dT_nominal</code> across the heat exchanger, within the limit of a
minimum mass flow rate ratio <code>ratFloMin</code>.
For computational performance and to avoid the use of a PI controller,
the required mass flow rate is computed based on a signal representative of
the load.
</p>
</html>",   revisions="<html>
<ul>
<li>
February 23, 2021, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"));
  end PrimaryVariableFlow;

  model Pump "Pump control"
    extends Modelica.Blocks.Icons.Block;
    parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
    parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
    parameter Real T0=273.15+20 "Nominal value for independent variable";
    parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
    parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
    Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Utilities.Math.CubicHermiteInverse spl(
      final xMin=TMin,
      final xMax=TMax,
      final x0=T0)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(final uMax=TMax, final
        uMin=TMin)
      annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    Modelica.Blocks.Interfaces.RealInput TMea "Measured temperature"
      annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    Modelica.Blocks.Interfaces.RealInput y "Control signal"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Buildings.Controls.Continuous.LimPID conPID(k=0.1, Ti=60,
      yMin=1e-3)
      annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  equation
    connect(spl.x, TSet.u)
      annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
    connect(y, spl.y)
      annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
    connect(TSet.y, conPID.u_s)
      annotation (Line(points={{-18,0},{38,0}}, color={0,0,127}));
    connect(TMea, conPID.u_m)
      annotation (Line(points={{-120,-60},{50,-60},{50,-12}}, color={0,0,127}));
    connect(conPID.y, yOut)
      annotation (Line(points={{61,0},{80,0},{80,0},{110,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Pump;

  model Pump2 "Pump control"
    extends Modelica.Blocks.Icons.Block;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Temperature TMin=288.15
      "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=298.15
      "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=293.15
      "Nominal value for independent variable";
    parameter Modelica.SIunits.TemperatureDifference dT=0.5
      "Temperature deadband for complete linear transition";
    parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
    parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
    Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Utilities.Math.CubicHermiteInverse spl(
      final xMin=TMin,
      final xMax=TMax,
      final x0=T0) if biomimeticControl
      "Spline to inversely calculate pulsing setpoint from control signal"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(
      final uMax=TMax,
      final uMin=TMin) if biomimeticControl
      "Temperature setpoint if biomimetic control"
      annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    Modelica.Blocks.Interfaces.RealInput TSetSta if not biomimeticControl
      "Static temperature setpoint"
      annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
    Modelica.Blocks.Interfaces.RealInput TMea
      "Measured temperature"
      annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
      "Control signal"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Buildings.Controls.OBC.CDL.Continuous.Line lin
      annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    Modelica.Blocks.Sources.Constant off(k=0.01) "Off signal"
      annotation (Placement(transformation(extent={{0,-50},{20,-30}})));
    Modelica.Blocks.Sources.Constant on(k=1) "On signal"
      annotation (Placement(transformation(extent={{0,50},{20,70}})));
    Modelica.Blocks.Sources.Constant dTSet(k=dT)   "Transition zone"
      annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
    Modelica.Blocks.Math.Add x1(k1=-1) "First transition point"
      annotation (Placement(transformation(extent={{0,10},{20,30}})));

  equation
    connect(spl.x, TSet.u)
      annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
    connect(y, spl.y)
      annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
    connect(TMea, lin.u) annotation (Line(points={{-120,-60},{50,-60},{50,0},{58,0}},
          color={0,0,127}));
    connect(off.y, lin.f2) annotation (Line(points={{21,-40},{40,-40},{40,-8},{58,
            -8}},
          color={0,0,127}));
    connect(TSet.y, x1.u2) annotation (Line(points={{-18,0},{-10,0},{-10,14},{-2,14}},
                  color={0,0,127}));
    connect(dTSet.y, x1.u1) annotation (Line(points={{-19,40},{-14,40},{-14,26},{-2,
            26}}, color={0,0,127}));
    connect(x1.y, lin.x1)
      annotation (Line(points={{21,20},{30,20},{30,8},{58,8}},color={0,0,127}));
    connect(on.y, lin.f1)
      annotation (Line(points={{21,60},{40,60},{40,4},{58,4}},color={0,0,127}));
    connect(TSet.y, lin.x2) annotation (Line(points={{-18,0},{-10,0},{-10,-4},{58,
            -4}},
          color={0,0,127}));
    connect(lin.y, yOut)
      annotation (Line(points={{82,0},{110,0}}, color={0,0,127}));
    connect(TSetSta, x1.u2) annotation (Line(points={{-120,60},{-70,60},{-70,20},{
            -10,20},{-10,14},{-2,14}}, color={0,0,127}));
    connect(TSetSta, lin.x2) annotation (Line(points={{-120,60},{-70,60},{-70,20},
            {-10,20},{-10,-4},{58,-4}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Pump2;

  model Pump3 "Pump control - on/off"
    extends Modelica.Blocks.Icons.Block;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Temperature TMin=288.15
      "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=298.15
      "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=293.15
      "Nominal value for independent variable";
    parameter Modelica.SIunits.TemperatureDifference dT=1
      "Temperature deadband for complete linear transition";
    parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
    parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
    Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Utilities.Math.CubicHermiteInverse spl(
      final xMin=TMin,
      final xMax=TMax,
      final x0=T0) if biomimeticControl
      "Spline to inversely calculate pulsing setpoint from control signal"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(
      final uMax=TMax,
      final uMin=TMin) if biomimeticControl
      "Temperature setpoint if biomimetic control"
      annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    Modelica.Blocks.Interfaces.RealInput TSetSta if not biomimeticControl
      "Static temperature setpoint"
      annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
    Modelica.Blocks.Interfaces.RealInput TMea
      "Measured temperature"
      annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
      "Control signal"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

    Buildings.Controls.OBC.CDL.Logical.OnOffController onOffCon(bandwidth=dT)
      annotation (Placement(transformation(extent={{20,-10},{40,10}})));
    Modelica.Blocks.Math.BooleanToReal booToRea(realFalse=0.02)
      "Convert boolean to real"
      annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  equation
    connect(spl.x, TSet.u)
      annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
    connect(y, spl.y)
      annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
    connect(TMea, onOffCon.u) annotation (Line(points={{-120,-60},{0,-60},{0,-6},
            {18,-6}}, color={0,0,127}));
    connect(TSet.y, onOffCon.reference)
      annotation (Line(points={{-18,0},{0,0},{0,6},{18,6}}, color={0,0,127}));
    connect(TSetSta, onOffCon.reference)
      annotation (Line(points={{-120,60},{0,60},{0,6},{18,6}}, color={0,0,127}));
    connect(onOffCon.y, booToRea.u)
      annotation (Line(points={{42,0},{58,0}}, color={255,0,255}));
    connect(booToRea.y, yOut)
      annotation (Line(points={{81,0},{92,0},{92,0},{110,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Pump3;
end Controls;
