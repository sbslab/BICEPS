within BICEPS.Fluid.Device.Controls;
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
