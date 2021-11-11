within BICEPS.Experimental.Examples;
package Sensors "Package with sensors for biomimetic control"
  extends Modelica.Icons.SensorsPackage;

  model RelativeElectricalExergyPotential
    "Sensor for relative electric potential exergy"
    extends Modelica.Icons.RotationalSensor;
    extends Buildings.BaseClasses.BaseIcon;
    parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Modelica.SIunits.Voltage v0=208 "Nominal value for independent variable";
    parameter Real k(min=Modelica.Constants.small)=10
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    final parameter Modelica.SIunits.Voltage vMin=v0*(1-tol) "Minimimum desired threshold for independent variable";
    final parameter Modelica.SIunits.Voltage vMax=v0*(1+tol) "Maximum desired threshold for independent variable";
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,-100}),     iconTransformation(extent={{-10,-110},{10,-90}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Sensors.Probe sen
      annotation (Placement(transformation(extent={{-10,20},{10,40}})));
    Modelica.Blocks.Interfaces.RealOutput y "Control signal"
      annotation (Placement(transformation(extent={{100,60},{120,80}})));
    Utilities.Math.CubicHermite spl(
      final xMin=vMin,
      final xMax=vMax,
      final x0=v0,
      final k=k,
      final ensureMonotonicity=true)
      annotation (Placement(transformation(extent={{60,60},{80,80}})));
  equation
    connect(terminal, terminal)
      annotation (Line(points={{0,-100},{0,-100}},
                                                 color={0,120,120}));
    connect(terminal, sen.term)
      annotation (Line(points={{0,-100},{0,21}},          color={0,120,120}));
    connect(sen.V, spl.x) annotation (Line(points={{7,33},{50,33},{50,70},{58,70}},
          color={0,0,127}));
    connect(spl.y, y)
      annotation (Line(points={{81,70},{110,70}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{60,-16},{-60,-66}},
            lineColor={0,0,0},
            textString="V"),
          Line(points={{18,58},{22,70}}, color={0,0,0}),
                             Line(points={{100,70},{22,70}},
                                                           color={0,0,0})}),
                                                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RelativeElectricalExergyPotential;

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
      final x0=p0,
      final k=k,
      final ensureMonotonicity=true)
      annotation (Placement(transformation(extent={{60,60},{80,80}})));
    Modelica.Blocks.Interfaces.RealOutput y "Control signal"
      annotation (Placement(transformation(extent={{100,60},{120,80}})));
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
      annotation (Line(points={{81,70},{110,70}}, color={0,0,127}));
    connect(senPre.p, spl.x) annotation (Line(points={{11,10},{48,10},{48,70},{58,
            70}}, color={0,0,127}));
    connect(port, senPre.port)
      annotation (Line(points={{0,-100},{0,0}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{62,-6},{-58,-56}},
            lineColor={0,0,0},
            textString="p"), Line(points={{100,70},{22,70}},
                                                           color={0,0,0}),
          Line(points={{18,58},{22,70}}, color={0,0,0})}),         Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RelativeFlowExergyPotential;

  model RelativeInternalExergyPotential
    "Sensor for relative internal exergy"
    extends Modelica.Icons.RotationalSensor;
    extends Buildings.Fluid.Interfaces.PartialTwoPort;
    parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
    parameter Real k(min=Modelica.Constants.small)=10
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
      "Nominal mass flow rate, used for regularization near zero flow"
      annotation(Dialog(group = "Nominal condition"));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          Medium, m_flow_nominal=m_flow_nominal)
      annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    Utilities.Math.CubicHermite spl(
      final xMin=TMin,
      final xMax=TMax,
      final x0=T0,
      final k=k,
      final ensureMonotonicity=true)
      annotation (Placement(transformation(extent={{-20,60},{0,80}})));
    Modelica.Blocks.Interfaces.RealOutput y "Control signal"
      annotation (Placement(transformation(extent={{0,100},{20,120}}),
          iconTransformation(
          extent={{100,60},{120,80}})));
  equation
    connect(port_a, senTem.port_a)
      annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
    connect(senTem.port_b, port_b)
      annotation (Line(points={{-40,0},{100,0}}, color={0,127,255}));
    connect(senTem.T, spl.x)
      annotation (Line(points={{-50,11},{-50,70},{-22,70}}, color={0,0,127}));
    connect(spl.y, y)
      annotation (Line(points={{1,70},{6,70},{6,110},{10,110}},
                                                 color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{-100,0},{-70,0}}, color={28,108,200}),
          Line(points={{70,0},{100,0}}, color={28,108,200}),
          Text(
            extent={{60,-18},{-60,-68}},
            lineColor={0,0,0},
            textString="T"),
          Line(points={{18,58},{22,70}}, color={0,0,0}),
                             Line(points={{100,70},{22,70}},
                                                           color={0,0,0})}),
                                                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RelativeInternalExergyPotential;
end Sensors;
