within BICEPS.Obsolete.Examples.Controls;
model SubsystemElectrical "Control for electrical subsystem"
  extends Modelica.Blocks.Icons.Block;
  parameter Integer n(min=1) "Number of input connectors";
  parameter Real a[n] = fill(1/n, n) "Weighting factors"
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
  Modelica.Blocks.Interfaces.RealVectorInput yIn[n] "Input control signals"
    annotation (Placement(transformation(extent={{-118,-20},{-78,20}})));
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Math.MultiSum multiSum(each k=a, nu=n)
    annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
  Modelica.Blocks.Math.Gain nor(k=sum(a)) "Normalized signal"
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
equation
  connect(multiSum.y, nor.u)
    annotation (Line(points={{-42.98,0},{-2,0}}, color={0,0,127}));
  connect(nor.y, yOut)
    annotation (Line(points={{21,0},{110,0}}, color={0,0,127}));
  connect(yIn, multiSum.u)
    annotation (Line(points={{-98,0},{-56,0}}, color={0,0,127}));
end SubsystemElectrical;
