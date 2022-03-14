within BICEPS.Experimental.Examples.Sensors;
model RelativeFlowExergyPotential "Sensor for relative flow exergy"
  extends Modelica.Icons.RotationalSensor;
  extends Buildings.BaseClasses.BaseIcon;
  replaceable package Medium =
    Modelica.Media.Interfaces.PartialMedium "Medium in the component"
      annotation (choices(
        choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
        choice(redeclare package Medium = Buildings.Media.Water "Water"),
        choice(redeclare package Medium =
            Buildings.Media.Antifreeze.PropyleneGlycolWater (
              property_T=293.15,
              X_a=0.40)
              "Propylene glycol water, 40% mass fraction")));
  parameter Real tol=0.1 "Tolerance allowed on nominal pressure control";
  parameter Modelica.SIunits.AbsolutePressure p0=101325 "Nominal value for independent variable";
  parameter Real k(min=Modelica.Constants.small)=10
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  final parameter Modelica.SIunits.AbsolutePressure pMin=p0*(1-tol) "Minimimum desired threshold for independent variable";
  final parameter Modelica.SIunits.AbsolutePressure pMax=p0*(1+tol) "Maximum desired threshold for independent variable";
  Utilities.Math.CubicHermite spl(
    final xMin=pMin,
    final xMax=pMax,
    final x0=p0)
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Interfaces.RealOutput y "Control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}}),
        iconTransformation(extent={{100,70},{120,90}})));
  Buildings.Fluid.Sensors.Pressure senPre(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  Modelica.Fluid.Interfaces.FluidPort_a port(redeclare package Medium = Medium,
      m_flow(min=0))
    annotation (Placement(transformation(
        origin={0,-100},
        extent={{-10,-10},{10,10}},
        rotation=90)));
equation
  connect(spl.y, y)
    annotation (Line(points={{81,70},{96,70},{96,80},{110,80}},
                                                color={0,0,127}));
  connect(senPre.p, spl.x) annotation (Line(points={{11,10},{48,10},{48,70},{58,
          70}}, color={0,0,127}));
  connect(port, senPre.port)
    annotation (Line(points={{0,-100},{0,0}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{62,-6},{-58,-56}},
          lineColor={0,0,0},
          textString="p"),
        Line(points={{18,58},{24,80}}, color={0,0,0}),
        Line(points={{0,-70},{0,-90}}, color={28,108,200}),
                           Line(points={{100,80},{24,80}},
                                                         color={0,0,0})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RelativeFlowExergyPotential;
