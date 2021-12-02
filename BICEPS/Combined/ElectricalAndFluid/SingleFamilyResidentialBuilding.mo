within BICEPS.Combined.ElectricalAndFluid;
model SingleFamilyResidentialBuilding
  "Combined thermofluid and electrical models for a single family residential building"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package MediumWat=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Medium in the building distribution system";
  replaceable package MediumAir=Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Load side medium";
  parameter Modelica.SIunits.Angle lat "Latitude";
  parameter Modelica.SIunits.Power PHeaPum_nominal=5000
    "Nominal power for heat pump";
  parameter Modelica.SIunits.Power PPum_nominal=100
    "Nominal power for pumps";
  parameter Modelica.SIunits.Power POth_nominal=10000
    "Nominal power for other loads";
  parameter Modelica.SIunits.Power PCon_nominal[3]=
    {PHeaPum_nominal,PPum_nominal,POth_nominal}
    "Nominal power for pumps";
  parameter Modelica.SIunits.Power PPV_nominal=4000
    "Nominal power for PV";
  parameter Modelica.SIunits.Power PBat_nominal=5800
    "Nominal power for battery";
  parameter String filNam
    "File name with thermal loads as time series";
  parameter Boolean allowFlowReversal=false
    "Set to true to allow flow reversal on condenser side"
    annotation (Dialog(tab="Assumptions"), Evaluate=true);
  Electrical.BuildingSystems.Electrical ele(
    nCon=3,
    lat=lat,
    PCon_nominal=PCon_nominal,
    PPro_nominal=PPV_nominal,
    PSto_nominal=PBat_nominal,
    EBatMax(displayUnit="kWh") = 48600000)          "Electrical subsystem"
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Fluid.BuildingSystems.ThermoFluidFourElements mec(
    redeclare package MediumWat = MediumWat,
    redeclare package MediumAir = MediumAir,
    QHea_flow_nominal=PHeaPum_nominal*mec.COP_nominal,
    COP_nominal=4)
    "Mechanical (thermo-fluid) subsystem"
    annotation (Placement(transformation(extent={{20,-40},{0,-20}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-20,80},{20,120}}),
        iconTransformation(extent={{-10,90},{10,110}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,80}),    iconTransformation(extent={{-120,60},{-100,80}})));
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare final package Medium = MediumWat,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
    h_outflow(start=MediumWat.h_default, nominal=MediumWat.h_default))
    "Fluid port inlet" annotation (Placement(transformation(extent={{90,-70},{110,
            -50}}),     iconTransformation(extent={{90,-70},{110,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare final package Medium = MediumWat,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
    h_outflow(start=MediumWat.h_default, nominal=MediumWat.h_default))
    "Fluid port outlet" annotation (Placement(transformation(extent={{-110,-70},
            {-90,-50}}),
                    iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Blocks.Sources.CombiTimeTable loaOth(
    tableOnFile=true,
    tableName="tab",
    fileName=Modelica.Utilities.Files.loadResource(filNam),
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    y(each unit="W"),
    columns={2},
    smoothness=Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1)
    "Reader for other electrical loads (combined lighting, devices, refrigerator, etc.)"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
equation
  connect(terminal, ele.terminal) annotation (Line(points={{-110,80},{-40,80},{-40,
          37},{-21,37}}, color={0,120,120}));
  connect(weaBus, ele.weaBus) annotation (Line(
      points={{0,100},{0,70},{-10,70},{-10,40}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, mec.weaBus) annotation (Line(
      points={{0,100},{0,70},{10,70},{10,-20}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(ele.yOut, mec.yEle) annotation (Line(points={{1,38},{30,38},{30,-23},{
          22,-23}}, color={0,0,127}));
  connect(mec.port_a, port_a) annotation (Line(points={{20,-36},{28,-36},{28,-60},
          {100,-60}}, color={0,127,255}));
  connect(mec.port_b, port_b) annotation (Line(points={{0,-36},{-10,-36},{-10,-60},
          {-100,-60}}, color={0,127,255}));
  connect(ele.yOut, yOut) annotation (Line(points={{1,38},{30,38},{30,80},{110,80}},
        color={0,0,127}));
  connect(mec.PHeaPum, ele.PCon[1]) annotation (Line(points={{-1,-25},{-40,-25},
          {-40,24},{-22,24},{-22,22.6667}}, color={0,0,127}));
  connect(mec.PPum, ele.PCon[2]) annotation (Line(points={{-1,-22},{-38,-22},{-38,
          22},{-24,22},{-24,24},{-22,24}}, color={0,0,127}));
  connect(loaOth.y[1], ele.PCon[3]) annotation (Line(points={{-59,30},{-40,30},
          {-40,25.3333},{-22,25.3333}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{0,70},{-60,40},{60,40},{0,70}},
          lineColor={95,95,95},
          smooth=Smooth.None,
          fillPattern=FillPattern.Solid,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-60,40},{60,-80}},
          lineColor={150,150,150},
          fillPattern=FillPattern.Sphere,
          fillColor={255,255,255}),
        Rectangle(
          extent={{20,2},{40,22}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38,0},{-18,20}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38,-60},{-18,-40}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{20,-60},{40,-40}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{60,-66},{100,-56}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-66},{-60,-56}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-60,-40},{-68,-12}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-60,-12},{-62,16}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-62,12},{-68,16}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
        Line(points={{-66,16}}, color={0,0,0}),
        Polygon(
          points={{-66,16},{-64,12},{-70,10},{-70,18},{-66,16}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-66,16},{-64,12}}, color={0,0,0}),
        Line(
          points={{-100,70},{-80,70},{-80,14},{-78,14},{-62,14}},
          color={0,140,72},
          thickness=0.5),
        Text(
          extent={{-94,64},{-46,40}},
          lineColor={0,140,72},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillPattern=FillPattern.Sphere,
          fillColor={0,140,72},
          textString="E"),
        Text(
          extent={{-98,-68},{-50,-92}},
          lineColor={28,108,200},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillPattern=FillPattern.Sphere,
          fillColor={0,140,72},
          textString="M")}),                                     Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SingleFamilyResidentialBuilding;
