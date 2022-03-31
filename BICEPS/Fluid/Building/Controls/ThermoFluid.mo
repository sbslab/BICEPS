within BICEPS.Fluid.Building.Controls;
model ThermoFluid "Thermofluid subsystem control block"
  extends Modelica.Blocks.Icons.Block;
  parameter Integer n(min=1) "Number of input connectors";
  parameter Real a[n] = fill(1/n, n) "Weighting factors";
  parameter Real tSmo(
    final quantity="Time",
    final unit="s",
    min=1E-5)=30*60
    "Smoothing time for thermal-fluid control signal";
  Modelica.Blocks.Math.MultiSum multiSum(each k=a, nu=n)
    annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
  Modelica.Blocks.Math.Gain nor(k=sum(a)) "Normalized signal"
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Modelica.Blocks.Interfaces.RealVectorInput yIn[n] "Input control signals"
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingMean smo(delta=tSmo)
    "Smoothing via a moving mean to account for thermal inertia"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  connect(multiSum.y,nor. u)
    annotation (Line(points={{-42.98,0},{-2,0}}, color={0,0,127}));
  connect(yIn,multiSum. u)
    annotation (Line(points={{-100,0},{-56,0}},color={0,0,127}));
  connect(nor.y, smo.u)
    annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
  connect(smo.y, yOut)
    annotation (Line(points={{62,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ThermoFluid;
